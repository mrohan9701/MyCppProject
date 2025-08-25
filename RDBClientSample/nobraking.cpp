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

// Flag to indicate if AEB controlled braking applied in this frame
bool aebApplied = false;
// Flag to disable built-in/internal braking and allow only AEB braking
bool disableBuiltInBraking = true;

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

void autonomousEmergencyBraking(VehState& ego, const VehState& frontObj,
                               float warningDistance, float emergencyDistance,
                               float maxDecelRate)
{
    aebApplied = false;  // reset at start of each call

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

int main(int argc, char* argv[])
{
    const char* serverIP = (argc > 1) ? argv[1] : "127.0.0.1";
    int port = (argc > 2) ? atoi(argv[2]) : DEFAULT_PORT;

    const float lane_window_m = 5.0f;
    const float fwd_max_range_m = 200.0f;

    const float warningDist = 30.0f;
    const float emergencyDist = 10.0f;
    const float maxDecelRate = 0.7f;

    int sClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sClient < 0) {
        perror("socket() failed");
        return 1;
    }

    struct sockaddr_in server;
    memset(&server, 0, sizeof(server));
    server.sin_family = AF_INET;
    server.sin_port   = htons(port);
    if (inet_pton(AF_INET, serverIP, &server.sin_addr) != 1) {
        perror("inet_pton failed");
        return 1;
    }

    if (connect(sClient, (struct sockaddr*)&server, sizeof(server)) < 0) {
        perror("connect() failed");
        return 1;
    }
    printf("Connected to RDB server %s:%d\n", serverIP, port);

    char* szBuffer = new char[DEFAULT_BUFFER];
    unsigned char* pData = (unsigned char*)calloc(1, sizeof(RDB_MSG_HDR_t));
    unsigned int bytesInBuffer = 0;
    size_t bufferSize = sizeof(RDB_MSG_HDR_t);

    // Store previous ego vx to ignore built-in braking if disabled
    float egoVxBeforeBraking = 0.0f;

    while (true)
    {
        int ret = recv(sClient, szBuffer, DEFAULT_BUFFER, 0);
        if (ret <= 0) {
            perror("recv() failed or closed");
            break;
        }

        if (bytesInBuffer + ret > bufferSize) {
            pData = (unsigned char*)realloc(pData, bytesInBuffer + ret);
            bufferSize = bytesInBuffer + ret;
        }
        memcpy(pData + bytesInBuffer, szBuffer, ret);
        bytesInBuffer += ret;

        while (bytesInBuffer >= sizeof(RDB_MSG_HDR_t)) {
            RDB_MSG_HDR_t* hdr = (RDB_MSG_HDR_t*)pData;
            if (hdr->magicNo != RDB_MAGIC_NO) {
                bytesInBuffer = 0;
                break;
            }

            unsigned int msgSize = hdr->headerSize + hdr->dataSize;
            if (bytesInBuffer < msgSize) break;

            VehState ego{};
            VehState bestFront{};
            float bestFrontDist = 1e9f;

            RDB_MSG_ENTRY_HDR_t* entry = (RDB_MSG_ENTRY_HDR_t*)((char*)hdr + hdr->headerSize);
            uint32_t remaining = hdr->dataSize;

            while (remaining) {
                if (entry->pkgId == RDB_PKG_ID_OBJECT_STATE) {
                    int noElements = entry->elementSize ? (entry->dataSize / entry->elementSize) : 0;
                    char* dataPtr = (char*)entry + entry->headerSize;

                    for (int i = 0; i < noElements; i++) {
                        RDB_OBJECT_STATE_t* obj = (RDB_OBJECT_STATE_t*)dataPtr;

                        VehState st{};
                        st.has = true;
                        st.id = obj->base.id;
                        strncpy(st.name, obj->base.name, sizeof(st.name) - 1);
                        st.category = obj->base.category;
                        st.type = obj->base.type;
                        st.x = obj->base.pos.x; st.y = obj->base.pos.y; st.z = obj->base.pos.z;
                        st.h = obj->base.pos.h; st.p = obj->base.pos.p; st.r = obj->base.pos.r;
                        st.vx = obj->ext.speed.x; st.vy = obj->ext.speed.y; st.vz = obj->ext.speed.z;
                        st.ax = obj->ext.accel.x; st.ay = obj->ext.accel.y; st.az = obj->ext.accel.z;

                        if (st.id == EGO_ID) {
                            ego = st;
                        }
                        dataPtr += entry->elementSize;
                    }
                }
                remaining -= (entry->headerSize + entry->dataSize);
                if (remaining)
                    entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
            }

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
                                        bestFront.category = obj->base.category;
                                        bestFront.type = obj->base.type;
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
                    if (remaining)
                        entry = (RDB_MSG_ENTRY_HDR_t*)((char*)entry + entry->headerSize + entry->dataSize);
                }
            }

            if (ego.has) {
                // If built-in braking is disabled, restore ego.vx to previous velocity before built-in braking applied
                static float lastEgoVx = 0.0f;
                if (disableBuiltInBraking) {
                    ego.vx = lastEgoVx;
                } else {
                    lastEgoVx = ego.vx;
                }

                printf("\n=== FRAME %u  t=%.3f ===\n", hdr->frameNo, hdr->simTime);
                printf("EGO (ID=%d, Name=%s)\n", ego.id, ego.name);
                printf("  Pos:  X=%.3f Y=%.3f Z=%.3f (m)\n", ego.x, ego.y, ego.z);
                printf("  Ori:  H=%.3f P=%.3f R=%.3f (rad)\n", ego.h, ego.p, ego.r);
                printf("  Vel:  Vx=%.3f Vy=%.3f Vz=%.3f (m/s)\n", ego.vx, ego.vy, ego.vz);
                printf("  Acc:  Ax=%.3f Ay=%.3f Az=%.3f (m/s^2)\n", ego.ax, ego.ay, ego.az);

                if (bestFront.has) {
                    float x_local, y_local;
                    worldToEgoLocal(bestFront.x, bestFront.y, ego.x, ego.y, ego.h, x_local, y_local);
                    float rel_vx = bestFront.vx - ego.vx;
                    float rel_vy = bestFront.vy - ego.vy;
                    float rel_speed = hypotf(rel_vx, rel_vy);
                    float dist_xy = hypotf(x_local, y_local);
                    float bearing = atan2f(y_local, x_local);

                    printf("Front Object (ID=%d, Name=%s)\n", bestFront.id, bestFront.name[0] ? bestFront.name : "N/A");
                    printf("  Pos:  X=%.3f Y=%.3f Z=%.3f (m)\n", bestFront.x, bestFront.y, bestFront.z);
                    printf("  Ori:  H=%.3f P=%.3f R=%.3f (rad)\n", bestFront.h, bestFront.p, bestFront.r);
                    printf("  Vel:  Vx=%.3f Vy=%.3f Vz=%.3f (m/s)\n", bestFront.vx, bestFront.vy, bestFront.vz);
                    printf("  Acc:  Ax=%.3f Ay=%.3f Az=%.3f (m/s^2)\n", bestFront.ax, bestFront.ay, bestFront.az);
                    printf("  Rel (ego frame): x_fwd=%.2f m  y_left=%.2f m  dist=%.2f m  bearing=%.2f rad\n",
                           x_local, y_local, dist_xy, bearing);
                    printf("  Rel speed (approx XY): %.3f m/s\n", rel_speed);
                } else {
                    printf("Front Object: none within +X, |y|<=%.1f m, range<=%.1f m\n",
                           lane_window_m, fwd_max_range_m);
                }

                // Apply autonomous emergency braking - only this function changes ego.vx!
                autonomousEmergencyBraking(ego, bestFront, warningDist, emergencyDist, maxDecelRate);

                // Update last known speed after AEB possible intervention
                lastEgoVx = ego.vx;

                if (aebApplied) {
                    printf("AEB is actively controlling ego vehicle speed this frame.\n");
                } else {
                    printf("No AEB braking active this frame.\n");
                }
            }

            memmove(pData, pData + msgSize, bytesInBuffer - msgSize);
            bytesInBuffer -= msgSize;
        }
    }

    close(sClient);
    delete[] szBuffer;
    free(pData);
    return 0;
}
