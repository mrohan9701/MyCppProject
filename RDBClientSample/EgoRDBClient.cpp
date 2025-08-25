#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>

#include "VtdToolkit/RDBHandler.hh"

#define DEFAULT_PORT        48190
#define DEFAULT_BUFFER      204800
#define EGO_ID              1

#ifndef SIM_DT
#define SIM_DT 0.05f
#endif

// Flag just for SIM_MODE logs
bool aebApplied = false;

struct VehState {
    bool has = false;
    int id = -1;
    char name[32] = {0};
    int category = 0;
    int type = 0;
    float x=0, y=0, z=0;
    float h=0, p=0, r=0;
    float vx=0, vy=0, vz=0;
    float ax=0, ay=0, az=0;
};

// Coordinate transform: world → ego
static inline void worldToEgoLocal(float x, float y, float ego_x, float ego_y, float ego_h,
                                   float& x_local, float& y_local)
{
    float dx = x - ego_x;
    float dy = y - ego_y;
    float ch = cosf(ego_h);
    float sh = sinf(ego_h);
    x_local =  ch*dx + sh*dy;
    y_local = -sh*dx + ch*dy;
}

// ---------- SEND DRIVER CTRL TO VTD ----------
static void sendDriverCtrl(int sock, double simTime, unsigned int simFrame, 
                           float accelTgt = 0.0f, float brakeTgt = 0.0f, float steerTgt = 0.0f,
                           bool stopNow = false) 
{
    Framework::RDBHandler handler;
    handler.initMsg();

    RDB_DRIVER_CTRL_t* drv =
        (RDB_DRIVER_CTRL_t*)handler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL, 1, sizeof(RDB_DRIVER_CTRL_t));
    if (!drv) return;

    memset(drv, 0, sizeof(RDB_DRIVER_CTRL_t));
    drv->playerId = EGO_ID;

    if (stopNow) {
        drv->accelTgt   = 0.0f;
        drv->brakePedal = 1.0f; 
        drv->steeringTgt = steerTgt;
        fprintf(stderr, "[CTRL] Full brake applied (brakePedal=1.0)\n");
    } else {
        drv->accelTgt   = accelTgt;
        drv->brakePedal = brakeTgt;
        drv->steeringTgt = steerTgt;
        fprintf(stderr, "[CTRL] Control (accel=%.2f, brake=%.2f, steer=%.2f)\n", accelTgt, brakeTgt, steerTgt);
    }

    drv->gear        = 1;

    drv->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING |
                         RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL |
                         RDB_DRIVER_INPUT_VALIDITY_BRAKE;

    int ret = send(sock, (const char*)handler.getMsg(), handler.getMsgTotalSize(), 0);
    if (ret <= 0) perror("sendDriverCtrl() send failed");
}

// --------- SIMPLE AEB (SIM MODE ONLY) ----------
void autonomousEmergencyBraking(VehState& ego, const VehState& frontObj,
                               float warningDistance, float emergencyDistance,
                               float maxDecelRate)
{
    aebApplied = false;
    if (!frontObj.has) return;

    float x_local, y_local;
    worldToEgoLocal(frontObj.x, frontObj.y, ego.x, ego.y, ego.h, x_local, y_local);
    float dist = hypotf(x_local, y_local);
    float rel_vx = frontObj.vx - ego.vx;
    if (rel_vx <= 0.0f) return;

    float ttc = dist / rel_vx;
    printf("TTC: %.3f s, Dist=%.3f m, Ego Vx=%.3f m/s\n", ttc, dist, ego.vx);

    const float ttc_warning = 3.0f;
    const float ttc_emergency = 1.0f;

    if (ttc <= ttc_emergency && dist <= emergencyDistance) {
        if (ego.vx > 0.0f) {
            printf("AEB Emergency brake! Ego stopped.\n");
            ego.vx = 0.0f;
            aebApplied = true;
        }
    } else if (ttc <= ttc_warning && dist <= warningDistance) {
        float decelFactor = maxDecelRate * (ttc_warning - ttc) / ttc_warning;
        if (decelFactor < 0.0f) decelFactor = 0.0f;
        if (decelFactor > 1.0f) decelFactor = 1.0f;
        float newVx = ego.vx * (1.0f - decelFactor);
        if (newVx < 0.0f) newVx = 0.0f;
        if (newVx < ego.vx) {
            printf("AEB Warning brake: Vx=%.3f → %.3f\n", ego.vx, newVx);
            ego.vx = newVx;
            aebApplied = true;
        }
    }
}

float laneFollowingSteering(float y_local, float laneCenterY, float kp = 0.5f)
{
    float error = y_local - laneCenterY;
    return -kp * error; // simple P control
}

#ifdef SIM_MODE
// ---------- SIMULATION ----------
void runSimulation()
{
    float ego_x = 0.0f, ego_y = 0.0f, ego_h = 0.0f, ego_vx = 20.0f;
    float front_x = 50.0f, front_y = 0.0f, front_vx = 0.0f;

    const float dt = SIM_DT;
    const float warningDist = 30.0f, emergencyDist = 10.0f, maxDecelRate = 0.7f;

    int frameNo = 0; float simTime = 0.0f;

    while (frameNo < 500) {
        ego_x += ego_vx * dt;

        VehState ego{}, front{};
        ego.has = true; ego.id = EGO_ID; strncpy(ego.name, "EgoVehicle", sizeof(ego.name)-1);
        ego.x = ego_x; ego.y = ego_y; ego.h = ego_h; ego.vx = ego_vx;

        front.has = true; front.id = 2; strncpy(front.name, "FrontObject", sizeof(front.name)-1);
        front.x = front_x; front.y = front_y; front.vx = front_vx;

        float x_local, y_local; worldToEgoLocal(front.x, front.y, ego.x, ego.y, ego.h, x_local, y_local);
        float dist = hypotf(x_local, y_local); float rel_v = fabsf(front.vx - ego.vx);

        printf("\n=== FRAME %d  t=%.3f ===\n", frameNo, simTime);
        printf("EGO: X=%.2f Vx=%.2f\n", ego.x, ego.vx);
        printf("Front: X=%.2f Vx=%.2f, Dist=%.2f, RelV=%.2f\n", front.x, front.vx, dist, rel_v);

        autonomousEmergencyBraking(ego, front, warningDist, emergencyDist, maxDecelRate);

        if (aebApplied) printf("AEB active this frame.\n");
        else            printf("No AEB braking.\n");

        ego_vx = ego.vx;
        if (ego_vx <= 0.01f && dist < 12.0f) { printf("Simulation stop: ego stopped near object.\n"); break; }

        frameNo++; simTime += dt; usleep((int)(dt * 1e6));
    }
}
#endif

// ============== MAIN ==============
int main(int argc, char* argv[])
{
#ifdef SIM_MODE
    printf("=== Running in SIM_MODE ===\n");
    runSimulation();
    return 0;
#else
    const char* serverIP = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : DEFAULT_PORT;
    printf("=== LIVE RDB MODE ===\nConnecting to VTD @ %s:%d ...\n", serverIP, port);

    // Parameters
    const float lane_window_m   = 5.0f;
    const float fwd_max_range_m = 200.0f;

    int sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sClient < 0) { perror("socket() failed"); return 1; }

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port   = htons(port);
    if (inet_pton(AF_INET, serverIP, &server.sin_addr) != 1) { perror("inet_pton failed"); return 1; }

    if (connect(sClient, (struct sockaddr*)&server, sizeof(server)) < 0) { perror("connect() failed"); return 1; }
    printf("Connected to RDB server.\n");

    char* szBuffer = new char[DEFAULT_BUFFER];
    unsigned char* pData = (unsigned char*)calloc(1, sizeof(RDB_MSG_HDR_t));
    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);

    while (true)
    {
        int ret = recv(sClient, szBuffer, DEFAULT_BUFFER, 0);
        if (ret <= 0) { perror("recv failed or closed"); break; }

        if (bytesInBuffer + ret > bufferSize) {
            pData = (unsigned char*)realloc(pData, bytesInBuffer + ret);
            bufferSize = bytesInBuffer + ret;
        }
        memcpy(pData + bytesInBuffer, szBuffer, ret);
        bytesInBuffer += ret;

        while (bytesInBuffer >= sizeof(RDB_MSG_HDR_t)) {
            RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*)pData;
            if (hdr->magicNo != RDB_MAGIC_NO) { bytesInBuffer = 0; break; }

            unsigned int msgSize = hdr->headerSize + hdr->dataSize;
            if (bytesInBuffer < msgSize) break;

            VehState ego{}, bestFront{};
            float bestFrontDist = 1e9f;

            // Pass 1: ego
            RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)((char*)hdr + hdr->headerSize);
            uint32_t remaining = hdr->dataSize;
            while (remaining) {
                if (entry->pkgId == RDB_PKG_ID_OBJECT_STATE) {
                    int noElements = entry->elementSize ? (entry->dataSize / entry->elementSize) : 0;
                    char* dataPtr = (char*)entry + entry->headerSize;
                    for (int i = 0; i < noElements; i++) {
                        RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)dataPtr;
                        if (obj->base.id == EGO_ID) {
                            ego.has = true; ego.id = obj->base.id;
                            strncpy(ego.name, obj->base.name, sizeof(ego.name)-1);
                            ego.x = obj->base.pos.x; ego.y = obj->base.pos.y; ego.z = obj->base.pos.z;
                            ego.h = obj->base.pos.h; ego.p = obj->base.pos.p; ego.r = obj->base.pos.r;
                            ego.vx = obj->ext.speed.x; ego.vy = obj->ext.speed.y; ego.vz = obj->ext.speed.z;
                            ego.ax = obj->ext.accel.x; ego.ay = obj->ext.accel.y; ego.az = obj->ext.accel.z;
                        }
                        dataPtr += entry->elementSize;
                    }
                }
                remaining -= (entry->headerSize + entry->dataSize);
                if (remaining) entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
            }

            // Pass 2: front object
            if (ego.has) {
                entry = (RDB_MSG_ENTRY_HDR_t*)((char*)hdr + hdr->headerSize);
                remaining = hdr->dataSize;
                while (remaining) {
                    if (entry->pkgId == RDB_PKG_ID_OBJECT_STATE) {
                        int noElements = entry->elementSize ? (entry->dataSize / entry->elementSize) : 0;
                        char* dataPtr = (char*)entry + entry->headerSize;
                        for (int i = 0; i < noElements; i++) {
                            RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)dataPtr;
                            if (obj->base.id != EGO_ID) {
                                float x_local, y_local;
                                worldToEgoLocal(obj->base.pos.x, obj->base.pos.y, ego.x, ego.y, ego.h, x_local, y_local);
                                if (x_local > 0.0f && fabsf(y_local) <= lane_window_m && x_local <= fwd_max_range_m) {
                                    float dist = hypotf(x_local, y_local);
                                    if (dist < bestFrontDist) {
                                        bestFrontDist = dist;
                                        bestFront.has = true;
                                        bestFront.id = obj->base.id;
                                        strncpy(bestFront.name, obj->base.name, sizeof(bestFront.name)-1);
                                        bestFront.x = obj->base.pos.x; bestFront.y = obj->base.pos.y; bestFront.vx = obj->ext.speed.x;
                                    }
                                }
                            }
                            dataPtr += entry->elementSize;
                        }
                    }
                    remaining -= (entry->headerSize + entry->dataSize);
                    if (remaining) entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
                }
            }

            // CONTROL DECISION
            if (ego.has) {
                float accelCmd = 0.0f, brakeCmd = 0.0f, steerCmd = 0.0f;

                if (bestFront.has) {
                    float x_local, y_local;
                    worldToEgoLocal(bestFront.x, bestFront.y, ego.x, ego.y, ego.h, x_local, y_local);
                    float dist_xy = hypotf(x_local, y_local);

                    if (dist_xy < 8.0f) { // danger zone
                        accelCmd = 0.0f; brakeCmd = 1.0f;
                        printf("[AEB] FULL BRAKE, d=%.2f\n", dist_xy);
                    }
                    else if (dist_xy < 20.0f) {
                        accelCmd = 0.0f; brakeCmd = (20.0f - dist_xy) / 20.0f;
                        printf("[AEB] Mod. brake=%.2f, d=%.2f\n", brakeCmd, dist_xy);
                    }
                    else {
                        accelCmd = 0.2f; brakeCmd = 0.0f;
                        printf("[CREEP] dist=%.2f → gentle accel=%.2f\n", dist_xy, accelCmd);
                    }
                } else {
                    accelCmd = 0.3f; brakeCmd = 0.0f;
                    printf("No vehicle ahead → cruise accel=%.2f\n", accelCmd);
                }

                sendDriverCtrl(sClient, hdr->simTime, hdr->frameNo, accelCmd, brakeCmd, steerCmd);
            }

            memmove(pData, pData + msgSize, bytesInBuffer - msgSize);
            bytesInBuffer -= msgSize;
        }
    }

    close(sClient);
    delete[] szBuffer;
    free(pData);
    return 0;
#endif
}
