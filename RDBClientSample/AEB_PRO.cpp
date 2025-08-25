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

// ---------- NEW: send driver control to VTD ----------
static void sendDriverCtrl(int sock, double simTime, unsigned int simFrame, 
                           float accelTgt = 0.0f, float brakeTgt = 0.0f, float steerTgt = 0.0f,
                           bool stopNow = false ) 
{
    Framework::RDBHandler handler;
    handler.initMsg();

    RDB_DRIVER_CTRL_t* drv =
        (RDB_DRIVER_CTRL_t*)handler.addPackage(simTime, simFrame, RDB_PKG_ID_DRIVER_CTRL, 1, sizeof(RDB_DRIVER_CTRL_t));
    if (!drv) return;

    memset(drv, 0, sizeof(RDB_DRIVER_CTRL_t));
    drv->playerId = EGO_ID;

    if (stopNow) {
        drv->accelTgt   = 0.0f;   // no throttle
        drv->brakePedal = 1.0f; 
        drv->speedTgt = 0;
        drv->steeringTgt = steerTgt;  // full brake
        fprintf(stderr, "[CTRL] Full brake applied (accelTgt=%.2f, brakePedal=%.2f)\n", accelTgt, brakeTgt);
    } else {
        drv->accelTgt   = accelTgt;  // normal throttle
        drv->brakePedal = brakeTgt;  // normal brake
        //drv->speedTgt = speedTgt;
        fprintf(stderr, "[CTRL] Normal control (accelTgt=%.2f, brakePedal=%.2f)\n", accelTgt, brakeTgt);
    }

    drv->steeringTgt = steerTgt;
    drv->gear        = 1;

    // mark which inputs are valid
    drv->validityFlags = RDB_DRIVER_INPUT_VALIDITY_TGT_STEERING |
                         RDB_DRIVER_INPUT_VALIDITY_TGT_ACCEL |
                         RDB_DRIVER_INPUT_VALIDITY_BRAKE | RDB_DRIVER_INPUT_VALIDITY_GEAR ;

    int ret = send(sock, (const char*)handler.getMsg(), handler.getMsgTotalSize(), 0);
    if (ret <= 0) perror("sendDriverCtrl() send failed");
}

// (Kept for SIM_MODE only; not used in live RDB control)
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
    printf("TTC: %.3f s, Distance: %.3f m, Ego Vx: %.3f m/s\n", ttc, dist, ego.vx);

    const float ttc_warning = 3.0f;
    const float ttc_emergency = 1.0f;

    if (ttc <= ttc_emergency && dist <= emergencyDistance) {
        if (ego.vx > 0.0f) {
            printf("AEB Emergency brake! Speed reduced from %.3f to 0.000 m/s\n", ego.vx);
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
            printf("AEB Warning brake: Speed reduced from %.3f to %.3f m/s\n", ego.vx, newVx);
            ego.vx = newVx;
            aebApplied = true;
        }
    }
}
float laneFollowingSteering(float y_local, float laneCenterY,
                            float heading, float laneHeading,
                            float kp = 0.5f, float kd = 0.2f)
{
    float y_error = y_local - laneCenterY;
    float heading_error = heading - laneHeading;

    // simple PD controller
    float steerCmd = -kp * y_error - kd * heading_error;
    return steerCmd;
}
#ifdef SIM_MODE
// Simple local sim (no RDB). Not used for real braking in VTD.
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
        printf("EGO: X=%.2f Vx=%.2f m/s\n", ego.x, ego.vx);
        printf("Front: X=%.2f Vx=%.2f m/s  Dist=%.2f m  RelV=%.2f m/s\n", front.x, front.vx, dist, rel_v);

        autonomousEmergencyBraking(ego, front, warningDist, emergencyDist, maxDecelRate);

        if (aebApplied) printf("AEB is actively controlling ego vehicle speed this frame.\n");
        else            printf("No AEB braking active this frame.\n");

        ego_vx = ego.vx;
        if (ego_vx <= 0.01f && dist < 12.0f) { printf("Simulation stop: ego stopped near object.\n"); break; }

        frameNo++; simTime += dt; usleep((int)(dt * 1e6));
    }
}
#endif

int main(int argc, char* argv[])
{
#ifdef SIM_MODE
    printf("=== Running in SIM_MODE (no VTD connection, using built-in simulation) ===\n");
    runSimulation();
    return 0;
#else
    const char* serverIP = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : DEFAULT_PORT;
    printf("=== Running in LIVE RDB MODE ===\n");
    printf("Connecting to VTD RDB server at %s:%d ...\n", serverIP, port);
    // Detection gate for “in front”
    const float lane_window_m   = 5.0f;    // |y_local| must be <= this
    const float fwd_max_range_m = 20.0f;  // ahead within this range
    const float kp_steer = 1.5f;
    const float kd_steer= 0.5f;
    const float laneCenterY = 0.0f;
    // float heading_error = ego.h - laneHeading; // if you have it
    // float steerCmd = -kp_steer * y_error - kd_steer * heading_error;




    int sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sClient < 0) { perror("socket() failed"); return 1; }

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port   = htons(port);
    if (inet_pton(AF_INET, serverIP, &server.sin_addr) != 1) { perror("inet_pton failed"); return 1; }

    if (connect(sClient, (struct sockaddr*)&server, sizeof(server)) < 0) { perror("connect() failed"); return 1; }
    printf("Connected to RDB server %s:%d\n", serverIP, port);

    char* szBuffer = new char[DEFAULT_BUFFER];
    unsigned char* pData = (unsigned char*)calloc(1, sizeof(RDB_MSG_HDR_t));
    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);

    while (true)
    {
        int ret = recv(sClient, szBuffer, DEFAULT_BUFFER, 0);
        if (ret <= 0) { perror("recv() failed or closed"); break; }

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

            VehState ego{};
            VehState bestFront{};
            float bestFrontDist = 1e9f;

            // 1st pass: find ego
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
                            strncpy(ego.name, obj->base.name, sizeof(ego.name) - 1);
                            ego.category = obj->base.category; ego.type = obj->base.type;
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

            // 2nd pass: find nearest “in-front” target
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

                                if (x_local > 0.0f &&
                                    fabsf(y_local) <= lane_window_m &&
                                    x_local <= fwd_max_range_m)
                                {
                                    float dist = hypotf(x_local, y_local);
                                    if (dist < bestFrontDist) {
                                        bestFrontDist = dist;
                                        bestFront.has = true;
                                        bestFront.id = obj->base.id;
                                        strncpy(bestFront.name, obj->base.name, sizeof(bestFront.name) - 1);
                                        bestFront.category = obj->base.category; bestFront.type = obj->base.type;
                                        bestFront.x = obj->base.pos.x; bestFront.y = obj->base.pos.y; bestFront.z = obj->base.pos.z;
                                        bestFront.h = obj->base.pos.h; bestFront.p = obj->base.pos.p; bestFront.r = obj->base.pos.r;
                                        bestFront.vx = obj->ext.speed.x; bestFront.vy = obj->ext.speed.y; bestFront.vz = obj->ext.speed.z;
                                        bestFront.ax = obj->ext.accel.x; bestFront.ay = obj->ext.accel.y; bestFront.az = obj->ext.accel.z;
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

            // Logs & CONTROL
            if (ego.has) {
                printf("\n=== FRAME %u  t=%.3f ===\n", hdr->frameNo, hdr->simTime);
                printf("EGO (ID=%d, Name=%s)\n", ego.id, ego.name);
                printf("  Pos:  X=%.3f Y=%.3f Z=%.3f (m)\n", ego.x, ego.y, ego.z);
                printf("  Ori:  H=%.3f P=%.3f R=%.3f (rad)\n", ego.h, ego.p, ego.r);
                printf("  Vel:  Vx=%.3f Vy=%.3f Vz=%.3f (m/s)\n", ego.vx, ego.vy, ego.vz);

                float accelCmd = 0.0f; // default: no accel/brake
                float brakeCmd = 0.0f;
                float steerCmd = laneFollowingSteering(ego.y, laneCenterY, ego.h, 0.0f, 1.5f, 0.5f);
                //float speedCmd = 5.0f;
                if (bestFront.has) {
                        autonomousEmergencyBraking(ego, bestFront, 30.0f, 10.0f, 0.7f);

              
                    float x_local, y_local;
                    worldToEgoLocal(bestFront.x, bestFront.y, ego.x, ego.y, ego.h, x_local, y_local);
                    float dist_xy = hypotf(x_local, y_local);
                    printf("Front Object (ID=%d, Name=%s)  dist=%.2f m, x_fwd=%.2f, y_left=%.2f\n",
                           bestFront.id, bestFront.name[0] ? bestFront.name : "N/A", dist_xy, x_local, y_local);

                    // ----------- SIMPLE RULE: if anything in front, FULL BRAKE -----------
                     accelCmd = -10.0f;
                    brakeCmd = 1.0f;
                    printf("Command: FULL BRAKE (accelTgt=%.2f) (brakeCmd=%.2f\n", accelCmd, brakeCmd);
                     } else {
                            float targetSpeed = 60.0f / 3.6f; // 50 km/h in m/s

                            if (ego.vx < targetSpeed) {
                                accelCmd = 1.0f;   // gentle acceleration
                                brakeCmd = 0.0f;
                                printf("CRUISE: accelerating (vx=%.2f m/s, target=%.2f)\n", ego.vx, targetSpeed);
                            } else {
                                accelCmd = 0.0f;   // stop accelerating, hold speed
                                brakeCmd = 0.0f;
                                printf("CRUISE: holding speed (vx=%.2f m/s)\n", ego.vx);
                            }  
                }
                 const float kp_steer = 0.5f;  // proportional gain
                 float y_error = ego.y - laneCenterY;
                float steerTgt = -kp_steer * y_error;
                 printf("Lane following: y_error=%.3f → steerCmd=%.3f\n", y_error, steerTgt);

                // >>> Send control to VTD <<<
                sendDriverCtrl(sClient, hdr->simTime, hdr->frameNo, accelCmd,brakeCmd);
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
