#include <GL/gl3w.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>

#include "shader.h"
#include "OBJLoader.h"

// Window Size
int WIN_W = 800, WIN_H = 600;

void framebuffer_size_callback(GLFWwindow*, int w, int h) {
    WIN_W = w; WIN_H = h;
    glViewport(0, 0, w, h);
}

//Shadow Map
const int    SHADOW_W = 4096;
const int    SHADOW_H = 4096;
unsigned int g_shadowFBO = 0;
unsigned int g_shadowTex = 0;
unsigned int g_shadowShader = 0;

const glm::vec3 SUN_DIR = glm::normalize(glm::vec3(0.4f, 1.0f, 0.3f));
const glm::vec3 SUN_COLOR = glm::vec3(1.0f, 0.95f, 0.80f);
const float     AMBIENT = 0.25f;

const float SHADOW_NEAR = 1.0f;
const float SHADOW_FAR = 300.0f;
const float SHADOW_ORTHO = 80.0f;

glm::mat4 g_lightSpaceMatrix(1.0f);

// OBJ Meshes
OBJMesh g_obs1Mesh;
OBJMesh g_obs2Mesh;

//Unit cube geometry
float unitCube[] = {
    -0.5f,-0.5f,-0.5f, 1,1,1,1,  0.5f,-0.5f,-0.5f, 1,1,1,1,  0.5f, 0.5f,-0.5f, 1,1,1,1,
     0.5f, 0.5f,-0.5f, 1,1,1,1, -0.5f, 0.5f,-0.5f, 1,1,1,1, -0.5f,-0.5f,-0.5f, 1,1,1,1,
    -0.5f,-0.5f, 0.5f, 1,1,1,1,  0.5f,-0.5f, 0.5f, 1,1,1,1,  0.5f, 0.5f, 0.5f, 1,1,1,1,
     0.5f, 0.5f, 0.5f, 1,1,1,1, -0.5f, 0.5f, 0.5f, 1,1,1,1, -0.5f,-0.5f, 0.5f, 1,1,1,1,
    -0.5f, 0.5f, 0.5f, 1,1,1,1, -0.5f, 0.5f,-0.5f, 1,1,1,1, -0.5f,-0.5f,-0.5f, 1,1,1,1,
    -0.5f,-0.5f,-0.5f, 1,1,1,1, -0.5f,-0.5f, 0.5f, 1,1,1,1, -0.5f, 0.5f, 0.5f, 1,1,1,1,
     0.5f, 0.5f, 0.5f, 1,1,1,1,  0.5f, 0.5f,-0.5f, 1,1,1,1,  0.5f,-0.5f,-0.5f, 1,1,1,1,
     0.5f,-0.5f,-0.5f, 1,1,1,1,  0.5f,-0.5f, 0.5f, 1,1,1,1,  0.5f, 0.5f, 0.5f, 1,1,1,1,
    -0.5f,-0.5f,-0.5f, 1,1,1,1,  0.5f,-0.5f,-0.5f, 1,1,1,1,  0.5f,-0.5f, 0.5f, 1,1,1,1,
     0.5f,-0.5f, 0.5f, 1,1,1,1, -0.5f,-0.5f, 0.5f, 1,1,1,1, -0.5f,-0.5f,-0.5f, 1,1,1,1,
    -0.5f, 0.5f,-0.5f, 1,1,1,1,  0.5f, 0.5f,-0.5f, 1,1,1,1,  0.5f, 0.5f, 0.5f, 1,1,1,1,
     0.5f, 0.5f, 0.5f, 1,1,1,1, -0.5f, 0.5f, 0.5f, 1,1,1,1, -0.5f, 0.5f,-0.5f, 1,1,1,1,
};

float groundVerts[] = {
     200,-0.5f, 200,  0.18f,0.18f,0.20f,1,
    -200,-0.5f, 200,  0.18f,0.18f,0.20f,1,
    -200,-0.5f,-200,  0.18f,0.18f,0.20f,1,
     200,-0.5f, 200,  0.18f,0.18f,0.20f,1,
    -200,-0.5f,-200,  0.18f,0.18f,0.20f,1,
     200,-0.5f,-200,  0.18f,0.18f,0.20f,1,
};

int g_score = 0;
int g_highScore = 0;

// Road System
struct RoadSegment {
    glm::vec2 center;
    float     heading;
};

const int   ROAD_SEG_LEN = 3;
const int   VISIBLE_SEGS = 130;
const float ROAD_WIDTH = 9.0f;

const float MAX_CURVE_NUDGE = 0.006f;
const float CURVE_BURST_MAG = 0.018f;
const int   BURST_MIN_LEN = 20;
const int   BURST_MAX_LEN = 50;
const float BURST_CHANCE = 0.008f;

std::vector<RoadSegment> roadSegs;
float roadHeading = 0.0f;

bool  inBurst = false;
int   burstRemain = 0;
float burstDir = 1.0f;

std::mt19937 rng(42);
std::uniform_real_distribution<float> unitDist(0.0f, 1.0f);
std::uniform_real_distribution<float> nudgeDist(-MAX_CURVE_NUDGE, MAX_CURVE_NUDGE);

// Car System + State
glm::vec3 carPos(0.0f);
glm::vec3 velocity(0.0f);
float     carAngle = 0.0f;

const float engineAccel = 0.022f;
const float brakeDecel = 0.028f;
const float maxSpeed = 0.55f;
const float rollingFriction = 0.990f;
const float tyreGrip = 0.08f;
const float maxTurnPerFrame = 1.5f;

int g_lives = 3;
const int MAX_LIVES = 3;

float nextHeadingDelta() {
    if (!inBurst && unitDist(rng) < BURST_CHANCE) {
        inBurst = true;
        burstRemain = BURST_MIN_LEN + (int)(unitDist(rng) * (BURST_MAX_LEN - BURST_MIN_LEN));
        burstDir = (unitDist(rng) < 0.5f) ? 1.0f : -1.0f;
    }
    float delta = nudgeDist(rng);
    if (inBurst) {
        delta += burstDir * CURVE_BURST_MAG;
        burstRemain--;
        if (burstRemain <= 0) inBurst = false;
    }
    delta += -roadHeading * 0.04f;
    return delta;
}

void initRoad() {
    roadSegs.clear();
    glm::vec2 cur(0.0f, 0.0f);
    for (int i = 0; i < VISIBLE_SEGS; ++i) {
        RoadSegment seg;
        seg.center = cur;
        seg.heading = roadHeading;
        roadSegs.push_back(seg);
        roadHeading += nextHeadingDelta();
        roadHeading = glm::clamp(roadHeading, -0.7f, 0.7f);
        cur.x += sinf(roadHeading) * ROAD_SEG_LEN;
        cur.y -= cosf(roadHeading) * ROAD_SEG_LEN;
    }
}

void extendRoad(float carZ) {
    while (roadSegs.size() > 2) {
        auto& front = roadSegs.front();
        float dz = front.center.y - carZ;
        if (dz > ROAD_SEG_LEN * 20) roadSegs.erase(roadSegs.begin());
        else break;
    }
    while ((int)roadSegs.size() < VISIBLE_SEGS) {
        auto& last = roadSegs.back();
        roadHeading += nextHeadingDelta();
        roadHeading = glm::clamp(roadHeading, -0.7f, 0.7f);
        glm::vec2 next;
        next.x = last.center.x + sinf(roadHeading) * ROAD_SEG_LEN;
        next.y = last.center.y - cosf(roadHeading) * ROAD_SEG_LEN;
        roadSegs.push_back({ next, roadHeading });
    }
}
// responible for smooth curvature of road and road barrier lines
glm::vec2 catmullRom(glm::vec2 p0, glm::vec2 p1, glm::vec2 p2, glm::vec2 p3, float t) {
    float t2 = t * t, t3 = t2 * t;
    return 0.5f * (
        (2.0f * p1) +
        (-p0 + p2) * t +
        (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
        (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}


bool getRoadAt(float worldZ, glm::vec2& outCenter, float& outHeading) {
    for (int i = 1; i + 2 < (int)roadSegs.size(); ++i) {
        float z0 = roadSegs[i].center.y;
        float z1 = roadSegs[i + 1].center.y;
        if (worldZ <= z0 && worldZ >= z1) {
            float t = (z0 - worldZ) / (z0 - z1 + 0.0001f);
            outCenter = catmullRom(roadSegs[i - 1].center, roadSegs[i].center,
                roadSegs[i + 1].center, roadSegs[i + 2].center, t);
            outHeading = glm::mix(roadSegs[i].heading, roadSegs[i + 1].heading, t);
            return true;
        }
    }
    return false;
}

// Building System
struct Building {
    glm::vec3 pos;
    float     height;
    float     r, g, b;
    bool      active;
};

const int   BUILD_POOL = 200;
const float BUILD_SIDE_OFFSET = 8.0f;
std::vector<Building> buildings;

glm::vec3 bCol(int idx) {
    switch (abs(idx) % 6) {
    case 0: return { 0.45f, 0.42f, 0.40f };
    case 1: return { 0.30f, 0.33f, 0.38f };
    case 2: return { 0.50f, 0.45f, 0.35f };
    case 3: return { 0.35f, 0.38f, 0.42f };
    case 4: return { 0.55f, 0.50f, 0.42f };
    default: return { 0.28f, 0.30f, 0.28f };
    }
}

void spawnBuildingAt(Building& b, int segIdx, int side) {
    if (segIdx < 0 || segIdx >= (int)roadSegs.size()) { b.active = false; return; }
    auto& seg = roadSegs[segIdx];
    float h = seg.heading;
    float px = cosf(h), pz = sinf(h);
    float ox = seg.center.x + side * (ROAD_WIDTH * 0.5f + BUILD_SIDE_OFFSET) * px;
    float oz = seg.center.y + side * (ROAD_WIDTH * 0.5f + BUILD_SIDE_OFFSET) * pz;
    float height = 0.8f + (abs(segIdx * 3 + side * 5) % 8) * 0.4f;
    glm::vec3 c = bCol(segIdx + side * 17);
    b.pos = glm::vec3(ox, 0, oz);
    b.height = height;
    b.r = c.r; b.g = c.g; b.b = c.b;
    b.active = true;
}

void initBuildings() {
    buildings.resize(BUILD_POOL);
    int half = BUILD_POOL / 2;
    for (int i = 0; i < half; ++i) {
        spawnBuildingAt(buildings[i], i % roadSegs.size(), -1);
        spawnBuildingAt(buildings[half + i], i % roadSegs.size(), +1);
    }
}

void updateBuildings(float carX, float carZ) {
    int half = BUILD_POOL / 2;
    static int nextLeft = 0;
    static int nextRight = 0;
    const float cullDist = ROAD_SEG_LEN * 30.0f;
    for (int i = 0; i < half; ++i) {
        auto& bl = buildings[i];
        auto& br = buildings[half + i];
        if (bl.active && (bl.pos.z - carZ) > cullDist) {
            int target = (int)roadSegs.size() - 1 - (nextLeft % 10);
            spawnBuildingAt(bl, target, -1);
            nextLeft++;
        }
        if (br.active && (br.pos.z - carZ) > cullDist) {
            int target = (int)roadSegs.size() - 1 - (nextRight % 10);
            spawnBuildingAt(br, target, +1);
            nextRight++;
        }
    }
}

// Tree System
struct Tree {
    glm::vec3 pos;
    float     scale;
    int       type;
    int       seed;
    bool      active;
};

const int   TREE_POOL = 300;
const float TREE_ROAD_OFFSET = 7.5f;
const float TREE_JITTER = 3.5f;
const int   TREE_SEG_SPACING = 2;

std::vector<Tree> trees;

bool isNearBuilding(glm::vec3 treePos, float minDist) {
    for (const auto& b : buildings) {
        if (!b.active) continue;
        float dx = treePos.x - b.pos.x;
        float dz = treePos.z - b.pos.z;
        if (dx * dx + dz * dz < minDist * minDist) return true;
    }
    return false;
}

void spawnTreeAt(Tree& t, int segIdx, int side, int seedVal) {
    if (segIdx < 0 || segIdx >= (int)roadSegs.size()) { t.active = false; return; }
    auto& seg = roadSegs[segIdx];
    float h = seg.heading;
    float perpX = cosf(h);
    float perpZ = sinf(h);
    float jitter = TREE_JITTER * ((seedVal % 100) / 100.0f - 0.5f);
    float lateral = side * (ROAD_WIDTH * 0.5f + TREE_ROAD_OFFSET + fabsf(jitter));
    t.pos.x = seg.center.x + lateral * perpX;
    t.pos.z = seg.center.y + lateral * perpZ;
    t.pos.y = -0.5f;

    if (isNearBuilding(t.pos, 1.5f)) {
        float forwardX = sinf(h);
        float forwardZ = -cosf(h);
        t.pos.x += forwardX * 4.0f;
        t.pos.z += forwardZ * 4.0f;
        if (isNearBuilding(t.pos, 1.5f)) { t.active = false; return; }
    }

    t.scale = 1.4f + (seedVal % 60) / 100.0f;
    t.type = abs(seedVal) % 3;
    if (t.type == 1) t.scale *= 0.8f;
    if (t.type == 2) t.scale *= 0.4f;
    t.seed = seedVal;
    t.active = true;

    glm::vec2 roadCenter; float rh;
    if (getRoadAt(t.pos.z, roadCenter, rh)) {
        float dx = t.pos.x - roadCenter.x;
        float dz = t.pos.z - roadCenter.y;
        float dist = sqrtf(dx * dx + dz * dz);
        if (dist < ROAD_WIDTH * 0.6f) { t.active = false; return; }
    }
}

void initTrees() {
    trees.resize(TREE_POOL);
    int half = TREE_POOL / 2;
    for (int i = 0; i < half; ++i) {
        int segIdx = i * TREE_SEG_SPACING + (i % 3);
        spawnTreeAt(trees[i], segIdx % (int)roadSegs.size(), -1, i * 37 + 13);
        spawnTreeAt(trees[half + i], segIdx % (int)roadSegs.size(), +1, i * 53 + 7);
    }
}

void updateTrees(float carZ) {
    int half = TREE_POOL / 2;
    static int nextLeft = 0;
    static int nextRight = 0;
    const float cullDist = ROAD_SEG_LEN * 30.0f;
    for (int i = 0; i < half; ++i) {
        auto& tl = trees[i];
        auto& tr = trees[half + i];
        if (tl.active && (tl.pos.z - carZ) > cullDist) {
            int target = (int)roadSegs.size() - 2 - (nextLeft % 8);
            spawnTreeAt(tl, target, -1, nextLeft * 41 + 3);
            nextLeft++;
        }
        if (tr.active && (tr.pos.z - carZ) > cullDist) {
            int target = (int)roadSegs.size() - 2 - (nextRight % 8);
            spawnTreeAt(tr, target, +1, nextRight * 67 + 19);
            nextRight++;
        }
    }
}

// Cloud System
struct Cloud {
    glm::vec3 pos;
    float     scale;
    int       seed;
    float     drift;
};

const int   CLOUD_COUNT = 20;
const float CLOUD_HEIGHT_MIN = 8.0f;
const float CLOUD_HEIGHT_MAX = 14.0f;
const float CLOUD_SPREAD = 120.0f;
const float CLOUD_ALPHA = 0.55f;

std::vector<Cloud> clouds;

void initClouds() {
    clouds.resize(CLOUD_COUNT);
    std::uniform_real_distribution<float> spreadDist(-CLOUD_SPREAD, CLOUD_SPREAD);
    std::uniform_real_distribution<float> hDist(CLOUD_HEIGHT_MIN, CLOUD_HEIGHT_MAX);
    std::uniform_real_distribution<float> sDist(2.5f, 5.5f);
    std::uniform_real_distribution<float> dDist(-0.008f, 0.008f);
    for (int i = 0; i < CLOUD_COUNT; ++i) {
        clouds[i].pos = glm::vec3(spreadDist(rng), hDist(rng), spreadDist(rng));
        clouds[i].scale = sDist(rng);
        clouds[i].seed = i * 73 + 11;
        clouds[i].drift = dDist(rng);
    }
}

void drawCloud(unsigned int shaderProg, unsigned int VAO,
    const Cloud& c, int uAlphaLoc)
{
    float x = c.pos.x, y = c.pos.y, z = c.pos.z;
    float sc = c.scale;
    int   sd = c.seed;

    float nudge = ((sd % 20) - 10) * 0.004f;
    float cr = 0.97f + nudge;
    float cg = 0.97f + nudge;
    float cb = 1.00f;

    auto puff = [&](glm::vec3 offset, glm::vec3 puffScale, float brightMod, float alpha) {
        glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(x, y, z) + offset);
        m = glm::scale(m, puffScale);
        glUniformMatrix4fv(glGetUniformLocation(shaderProg, "model"), 1, GL_FALSE, glm::value_ptr(m));
        glUniform3f(glGetUniformLocation(shaderProg, "uColor"),
            cr * brightMod, cg * brightMod, cb * brightMod);
        glUniform1f(uAlphaLoc, alpha);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        };

    puff({ 0,          0,          0 }, { sc * 3.0f, sc * 1.2f, sc * 1.8f }, 1.00f, CLOUD_ALPHA);

    float ox1 = ((sd % 7) - 3) * 0.3f * sc;
    float ox2 = ((sd % 11) - 5) * 0.2f * sc;
    float oz1 = ((sd % 5) - 2) * 0.3f * sc;

    puff({ sc * 1.4f + ox1,  sc * 0.3f,  oz1 }, { sc * 1.8f, sc * 1.0f, sc * 1.4f }, 0.98f, CLOUD_ALPHA * 0.90f);
    puff({ -sc * 1.6f + ox2, sc * 0.1f,  oz1 * 0.5f }, { sc * 1.6f, sc * 0.9f, sc * 1.3f }, 0.97f, CLOUD_ALPHA * 0.85f);
    puff({ ox1 * 0.5f,     sc * 0.7f,  sc * 0.8f }, { sc * 1.4f, sc * 0.8f, sc * 1.0f }, 0.99f, CLOUD_ALPHA * 0.80f);
    puff({ ox2 * 0.8f,     sc * 0.6f, -sc * 0.9f }, { sc * 1.3f, sc * 0.75f,sc * 1.1f }, 0.96f, CLOUD_ALPHA * 0.75f);
    puff({ 0,           -sc * 0.55f, 0 }, { sc * 2.5f, sc * 0.45f, sc * 1.5f }, 0.72f, CLOUD_ALPHA * 0.60f);
}

void updateClouds(const glm::vec3& cp) {
    const float RECYCLE_DIST = CLOUD_SPREAD * 1.5f;
    std::uniform_real_distribution<float> spreadDist(-CLOUD_SPREAD, CLOUD_SPREAD);
    std::uniform_real_distribution<float> hDist(CLOUD_HEIGHT_MIN, CLOUD_HEIGHT_MAX);
    for (auto& c : clouds) {
        c.pos.x += c.drift;
        float dx = c.pos.x - cp.x;
        float dz = c.pos.z - cp.z;
        if (dx * dx + dz * dz > RECYCLE_DIST * RECYCLE_DIST) {
            c.pos.x = cp.x + spreadDist(rng);
            c.pos.z = cp.z + spreadDist(rng);
            c.pos.y = hDist(rng);
        }
    }
}

// Onstacle System (obstacle1 -> cone, obstacle2 -> coin)
struct Obstacle {
    glm::vec3 pos;
    float     r, g, b;
    bool      active;
    float     width, depth, height;
    bool      scored = false;
    int       type;
};

const int OBS_POOL = 20;
const int OBS_SEG_STRIDE = 8;
std::vector<Obstacle> obstacles;

void spawnObstacle(Obstacle& obs, int segIdx) {
    if (segIdx < 0 || segIdx >= (int)roadSegs.size()) { obs.active = false; return; }
    auto& seg = roadSegs[segIdx];
    float h = seg.heading;
    float lateral = (unitDist(rng) - 0.5f) * ROAD_WIDTH * 0.7f;
    float px = cosf(h), pz = sinf(h);
    obs.pos.x = seg.center.x + lateral * px;
    obs.pos.z = seg.center.y + lateral * pz;
    obs.pos.y = 0.0f;
    obs.type = abs((int)(seg.center.x * 100 + segIdx)) % 2;
    if (obs.type == 0) {
        obs.r = 0.95f; obs.g = 0.45f; obs.b = 0.05f;
        obs.width = 0.5f; obs.depth = 0.5f; obs.height = 1.0f;
    }
    else {
        obs.r = 0.15f; obs.g = 0.55f; obs.b = 0.25f;
        obs.width = 0.8f; obs.depth = 0.8f; obs.height = 1.2f;
    }
    obs.active = true;
}

void initObstacles() {
    obstacles.resize(OBS_POOL);
    for (int i = 0; i < OBS_POOL; ++i)
        spawnObstacle(obstacles[i], (40 + i * OBS_SEG_STRIDE) % (int)roadSegs.size());
}

void updateObstacles(float carZ) {
    const float cull = ROAD_SEG_LEN * 20.0f;
    for (auto& obs : obstacles) {
        if (!obs.active) continue;
        if ((obs.pos.z - carZ) > cull) {
            int seg = (int)roadSegs.size() - 10 - (rng() % 5);
            if (seg >= 0 && seg < (int)roadSegs.size()) {
                spawnObstacle(obs, seg);
                obs.scored = false;
            }
            else {
                obs.active = false;
            }
        }
    }
}

// Game System + collision handling
bool  g_crashed = false;
float g_crashTimer = 0.0f;

bool boxOverlap(glm::vec3 aPos, glm::vec3 aHalf,
    glm::vec3 bPos, glm::vec3 bHalf) {
    return fabsf(aPos.x - bPos.x) < aHalf.x + bHalf.x &&
        fabsf(aPos.y - bPos.y) < aHalf.y + bHalf.y &&
        fabsf(aPos.z - bPos.z) < aHalf.z + bHalf.z;
}

void resetGame() {
    carPos = glm::vec3(0.0f);
    velocity = glm::vec3(0.0f);
    carAngle = 0.0f;
    g_crashed = false;
    g_crashTimer = 0.0f;
    g_highScore = std::max(g_highScore, g_score);
    g_score = 0;
    g_lives = MAX_LIVES;
    roadSegs.clear();
    roadHeading = 0.0f;
    initRoad();
    initObstacles();
    initBuildings();
    initTrees();
}

void processInput(GLFWwindow* win) {
    if (g_crashed) {
        velocity *= 0.92f;
        carPos += velocity;
        g_crashTimer -= 1.0f;
        if (g_crashTimer <= 0.0f) {
            g_crashed = false;
            velocity = glm::vec3(0.0f);
        }
        return;
    }

    float rad = glm::radians(carAngle);
    glm::vec3 forward(sinf(rad), 0.0f, -cosf(rad));
    glm::vec3 right(cosf(rad), 0.0f, sinf(rad));

    if (glfwGetKey(win, GLFW_KEY_W) == GLFW_PRESS) velocity += forward * engineAccel;
    if (glfwGetKey(win, GLFW_KEY_S) == GLFW_PRESS) velocity -= forward * brakeDecel;

    float speedFwd = glm::dot(velocity, forward);
    float speedNorm = fabsf(speedFwd) / maxSpeed;
    float steerDir = (speedFwd >= 0.0f) ? 1.0f : -1.0f;

    if (glfwGetKey(win, GLFW_KEY_A) == GLFW_PRESS) carAngle -= maxTurnPerFrame * speedNorm * steerDir;
    if (glfwGetKey(win, GLFW_KEY_D) == GLFW_PRESS) carAngle += maxTurnPerFrame * speedNorm * steerDir;

    rad = glm::radians(carAngle);
    forward = glm::vec3(sinf(rad), 0.0f, -cosf(rad));
    right = glm::vec3(cosf(rad), 0.0f, sinf(rad));

    glm::vec3 fwdVel = glm::dot(velocity, forward) * forward;
    glm::vec3 latVel = glm::dot(velocity, right) * right;
    velocity = fwdVel * rollingFriction + latVel * tyreGrip;

    if (glm::length(velocity) > maxSpeed)
        velocity = glm::normalize(velocity) * maxSpeed;

    carPos += velocity;

    glm::vec2 roadCenter; float rh;
    if (getRoadAt(carPos.z, roadCenter, rh)) {
        glm::vec2 rightVec(cosf(rh), sinf(rh));
        glm::vec2 toCar = glm::vec2(carPos.x, carPos.z) - roadCenter;
        if (fabs(glm::dot(toCar, rightVec)) > ROAD_WIDTH * 0.5f) resetGame();
    }

    glm::vec3 carHalf(0.40f, 0.20f, 0.80f);
    for (auto& obs : obstacles) {
        if (!obs.active) continue;
        glm::vec3 obsHalf(obs.width * 0.5f, obs.height * 0.5f, obs.depth * 0.5f);
        glm::vec3 obsCenter = obs.pos + glm::vec3(0, obs.height * 0.5f, 0);
        if (boxOverlap(carPos, carHalf, obsCenter, obsHalf)) {
            if (obs.type == 1) {
                if (!obs.scored) { g_score += 1; obs.scored = true; }
                obs.active = false;
                continue;
            }
            g_crashed = true;
            g_lives--;
            g_crashTimer = 40.0f;
            velocity *= -0.5f;
            break;
        }
    }
}
bool isCarOnRoad(const glm::vec3& cp) {
    for (auto& seg : roadSegs) {
        float dx = cp.x - seg.center.x;
        float dz = cp.z - seg.center.y;
        float h = seg.heading;
        glm::vec2 fwd = glm::vec2(sinf(h), -cosf(h));
        glm::vec2 rgt = glm::vec2(fwd.y, -fwd.x);
        glm::vec2 off(dx, dz);
        float lateral = fabsf(off.x * rgt.x + off.y * rgt.y);
        if (lateral < ROAD_WIDTH * 0.5f) return true;
    }
    return false;
}

// Draw Helpers
void drawCarPart(unsigned int shaderProg, unsigned int VAO,
    const glm::mat4& carT,
    glm::vec3 localPos, glm::vec3 scale,
    float r, float g, float b, float alpha = 1.0f)
{
    glm::mat4 m = glm::translate(carT, localPos);
    m = glm::scale(m, scale);
    glUniformMatrix4fv(glGetUniformLocation(shaderProg, "model"), 1, GL_FALSE, glm::value_ptr(m));
    glUniform3f(glGetUniformLocation(shaderProg, "uColor"), r, g, b);
    glUniform1f(glGetUniformLocation(shaderProg, "uAlpha"), alpha);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void drawBox(unsigned int shaderProg, unsigned int VAO,
    glm::vec3 pos, glm::vec3 scale,
    float r, float g, float b, float alpha = 1.0f)
{
    glm::mat4 m = glm::translate(glm::mat4(1.0f), pos);
    m = glm::scale(m, scale);
    glUniformMatrix4fv(glGetUniformLocation(shaderProg, "model"), 1, GL_FALSE, glm::value_ptr(m));
    glUniform3f(glGetUniformLocation(shaderProg, "uColor"), r, g, b);
    glUniform1f(glGetUniformLocation(shaderProg, "uAlpha"), alpha);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void drawOBJ(unsigned int shaderProg, const OBJMesh& mesh,
    const glm::mat4& modelMat, int colType)
{
    glm::mat4 finalModel = modelMat;

    if (colType == 0) {
        glUniform1i(glGetUniformLocation(shaderProg, "uUseVertexColor"), 0);
        glUniform3f(glGetUniformLocation(shaderProg, "uColor"), 1.0f, 0.5f, 0.0f);
    }
    else {

        glUniform1i(glGetUniformLocation(shaderProg, "uUseVertexColor"), 0);
        glUniform3f(glGetUniformLocation(shaderProg, "uColor"), 1.0f, 0.85f, 0.1f);
    }

    glUniform1f(glGetUniformLocation(shaderProg, "uAlpha"), 1.0f);

    glUniformMatrix4fv(
        glGetUniformLocation(shaderProg, "model"),
        1, GL_FALSE, glm::value_ptr(finalModel)
    );

    glBindVertexArray(mesh.VAO);
    glDrawArrays(GL_TRIANGLES, 0, mesh.vertexCount);
}

// Different Tree types for variety
void drawTree(unsigned int shaderProg, unsigned int VAO, const Tree& t) {
    float x = t.pos.x, z = t.pos.z, sc = t.scale;
    float baseY = t.pos.y;
    int sd = t.seed;
    float nudge = ((sd % 20) - 10) * 0.008f;

    if (t.type == 0) { //pine tree
        float bDr = 0.28f, bDg = 0.17f, bDb = 0.06f;
        float bLr = 0.42f, bLg = 0.27f, bLb = 0.10f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 0.45f, z }, { 0.28f * sc, sc * 0.90f, 0.28f * sc }, bDr, bDg, bDb);
        drawBox(shaderProg, VAO, { x, baseY + sc * 0.30f + 0.05f, z }, { 0.22f * sc, sc * 0.20f, 0.22f * sc }, bLr, bLg, bLb);
        drawBox(shaderProg, VAO, { x, baseY + sc * 0.65f + 0.05f, z }, { 0.18f * sc, sc * 0.20f, 0.18f * sc }, bLr, bLg, bLb);

        float g1r = 0.08f + nudge, g1g = 0.30f + nudge, g1b = 0.08f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 1.05f, z }, { 1.30f * sc, sc * 0.55f, 1.30f * sc }, g1r, g1g, g1b);
        drawBox(shaderProg, VAO, { x + 0.15f * sc, baseY + sc * 1.00f, z - 0.15f * sc }, { 0.55f * sc, sc * 0.30f, 0.55f * sc }, g1r * 0.75f, g1g * 0.75f, g1b * 0.75f);
        drawBox(shaderProg, VAO, { x - 0.20f * sc, baseY + sc * 1.05f, z + 0.10f * sc }, { 0.40f * sc, sc * 0.25f, 0.40f * sc }, g1r * 0.75f, g1g * 0.75f, g1b * 0.75f);

        float g2r = 0.10f + nudge, g2g = 0.38f + nudge, g2b = 0.10f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 1.60f, z }, { 1.00f * sc, sc * 0.50f, 1.00f * sc }, g2r, g2g, g2b);

        float g3r = 0.12f + nudge, g3g = 0.46f + nudge, g3b = 0.12f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 2.10f, z }, { 0.72f * sc, sc * 0.45f, 0.72f * sc }, g3r, g3g, g3b);
        drawBox(shaderProg, VAO, { x, baseY + sc * 2.55f, z }, { 0.38f * sc, sc * 0.40f, 0.38f * sc }, 0.25f + nudge, 0.60f + nudge, 0.20f);
    }

    else if (t.type == 1) { //big tree
        float bR = 0.35f, bG = 0.22f, bB = 0.08f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 0.55f, z }, { 0.38f * sc, sc * 1.10f, 0.38f * sc }, bR, bG, bB);

        float cr = 0.10f + nudge, cg = 0.38f + nudge, cb = 0.10f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 1.80f, z }, { 1.60f * sc, sc * 1.10f, 1.60f * sc }, cr, cg, cb);
        drawBox(shaderProg, VAO, { x, baseY + sc * 2.55f, z }, { 0.90f * sc, sc * 0.45f, 0.90f * sc }, 0.28f + nudge, 0.65f + nudge, 0.18f);
    }

    else { //small tree (bush)
        float stemR = 0.32f, stemG = 0.20f, stemB = 0.07f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 0.18f, z }, { 0.18f * sc, sc * 0.35f, 0.18f * sc }, stemR, stemG, stemB);

        float br = 0.12f + nudge, bg = 0.42f + nudge, bb = 0.10f;

        drawBox(shaderProg, VAO, { x, baseY + sc * 0.55f, z }, { 1.50f * sc, sc * 0.45f, 1.50f * sc }, br * 0.80f, bg * 0.80f, bb * 0.80f);
        drawBox(shaderProg, VAO, { x, baseY + sc * 0.85f, z }, { 1.20f * sc, sc * 0.40f, 1.20f * sc }, br, bg, bb);
    }
}


void buildSmoothedBarriers(std::vector<glm::vec3>& leftOut,
    std::vector<glm::vec3>& rightOut,
    int stepsPerSeg)
{
    leftOut.clear(); rightOut.clear();
    int n = (int)roadSegs.size();
    if (n < 4) return;
    for (int i = 1; i + 2 < n; ++i) {
        glm::vec2 p0 = roadSegs[i - 1].center, p1 = roadSegs[i].center;
        glm::vec2 p2 = roadSegs[i + 1].center, p3 = roadSegs[i + 2].center;
        float h0 = roadSegs[i - 1].heading, h1 = roadSegs[i].heading;
        float h2 = roadSegs[i + 1].heading, h3 = roadSegs[i + 2].heading;
        for (int s = 0; s < stepsPerSeg; ++s) {
            float t = float(s) / float(stepsPerSeg), t2 = t * t, t3 = t2 * t;
            glm::vec2 center = catmullRom(p0, p1, p2, p3, t);
            float h = 0.5f * (2.0f * h1 + (-h0 + h2) * t + (2.0f * h0 - 5.0f * h1 + 4.0f * h2 - h3) * t2 + (-h0 + 3.0f * h1 - 3.0f * h2 + h3) * t3);
            float px = cosf(h) * ROAD_WIDTH * 0.5f, pz = sinf(h) * ROAD_WIDTH * 0.5f;
            leftOut.push_back(glm::vec3(center.x - px, -0.47f, center.y - pz));
            rightOut.push_back(glm::vec3(center.x + px, -0.47f, center.y + pz));
        }
    }
}

struct RoadQuad { glm::vec3 leftA, rightA, leftB, rightB; };

void buildSmoothedRoadQuads(std::vector<RoadQuad>& quadsOut, int stepsPerSeg) {
    quadsOut.clear();
    int n = (int)roadSegs.size();
    if (n < 4) return;
    std::vector<glm::vec3> leftPts, rightPts;
    for (int i = 1; i + 2 < n; ++i) {
        glm::vec2 p0 = roadSegs[i - 1].center, p1 = roadSegs[i].center;
        glm::vec2 p2 = roadSegs[i + 1].center, p3 = roadSegs[i + 2].center;
        float h0 = roadSegs[i - 1].heading, h1 = roadSegs[i].heading;
        float h2 = roadSegs[i + 1].heading, h3 = roadSegs[i + 2].heading;
        int limit = (i + 3 < n) ? stepsPerSeg : stepsPerSeg + 1;
        for (int s = 0; s < limit; ++s) {
            float t = float(s) / float(stepsPerSeg), t2 = t * t, t3 = t2 * t;
            glm::vec2 center = catmullRom(p0, p1, p2, p3, t);
            float h = 0.5f * (2.0f * h1 + (-h0 + h2) * t + (2.0f * h0 - 5.0f * h1 + 4.0f * h2 - h3) * t2 + (-h0 + 3.0f * h1 - 3.0f * h2 + h3) * t3);
            float px = cosf(h) * ROAD_WIDTH * 0.5f, pz = sinf(h) * ROAD_WIDTH * 0.5f;
            leftPts.push_back(glm::vec3(center.x - px, -0.490f, center.y - pz));
            rightPts.push_back(glm::vec3(center.x + px, -0.490f, center.y + pz));
        }
    }
    for (int i = 0; i + 1 < (int)leftPts.size(); ++i)
        quadsOut.push_back({ leftPts[i], rightPts[i], leftPts[i + 1], rightPts[i + 1] });
}

// -------------------------------------------------------
int main()
{
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 5);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

    GLFWwindow* win = glfwCreateWindow(WIN_W, WIN_H, "CAR GAME", NULL, NULL);
    glfwMakeContextCurrent(win);
    glfwSetFramebufferSizeCallback(win, framebuffer_size_callback);
    gl3wInit();

    // ---- Shadow FBO setup ----
    glGenFramebuffers(1, &g_shadowFBO);
    glGenTextures(1, &g_shadowTex);
    glBindTexture(GL_TEXTURE_2D, g_shadowTex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT,
        SHADOW_W, SHADOW_H, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = { 1,1,1,1 };
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
    glBindFramebuffer(GL_FRAMEBUFFER, g_shadowFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, g_shadowTex, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    g_shadowShader = CompileShader("shadow.vert", "shadow.frag");

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_MULTISAMPLE);

    unsigned int shader = CompileShader("triangle.vert", "triangle.frag");
    int uAlphaLoc = glGetUniformLocation(shader, "uAlpha");

    // ---- VAOs ----
    unsigned int VAO[2], VBO[2];
    glGenVertexArrays(2, VAO);
    glGenBuffers(2, VBO);

    auto setupVAO = [&](int i, float* data, size_t bytes) {
        glBindVertexArray(VAO[i]);
        glBindBuffer(GL_ARRAY_BUFFER, VBO[i]);
        glBufferData(GL_ARRAY_BUFFER, bytes, data, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
        };
    setupVAO(0, unitCube, sizeof(unitCube));
    setupVAO(1, groundVerts, sizeof(groundVerts));

    if (!LoadOBJ("assets/obstacle1.obj", g_obs1Mesh)) std::cerr << "WARNING: obstacle1.obj not found\n";
    if (!LoadOBJ("assets/obstacle2.obj", g_obs2Mesh)) std::cerr << "WARNING: obstacle2.obj not found\n";
    UploadOBJ(g_obs1Mesh);
    UploadOBJ(g_obs2Mesh);

    glUseProgram(shader);
    glUniform1i(glGetUniformLocation(shader, "uUseVertexColor"), 0);
    glUniform1f(uAlphaLoc, 1.0f);

    initRoad();
    initBuildings();
    initTrees();
    initObstacles();
    initClouds();

    unsigned int dynVAO, dynVBO;
    glGenVertexArrays(1, &dynVAO);
    glGenBuffers(1, &dynVBO);

    unsigned int lineVAO, lineVBO;
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);

    // main render loop
    while (!glfwWindowShouldClose(win))
    {
        //Update
        processInput(win);
        if (!isCarOnRoad(carPos) || g_lives <= 0) resetGame();

        extendRoad(carPos.z);
        updateBuildings(carPos.x, carPos.z);
        updateObstacles(carPos.z);
        updateTrees(carPos.z);
        updateClouds(carPos);

        //Camera
        float camRad = glm::radians(carAngle);
        glm::vec3 camPos = carPos + glm::vec3(-sinf(camRad) * 7.0f, 3.5f, cosf(camRad) * 7.0f);
        glm::vec3 lookAt = carPos + glm::vec3(sinf(camRad) * 2.0f, 0.3f, -cosf(camRad) * 2.0f);
        glm::mat4 view = glm::lookAt(camPos, lookAt, glm::vec3(0, 1, 0));
        float aspect = (float)WIN_W / (float)WIN_H;
        glm::mat4 proj = glm::perspective(glm::radians(60.0f), aspect, 0.1f, 400.0f);

        // Light-space matrix (follows car) 
        glm::vec3 sunEye = carPos + SUN_DIR * 120.0f;
        glm::mat4 lightView = glm::lookAt(sunEye, carPos, glm::vec3(0, 1, 0));
        glm::mat4 lightProj = glm::ortho(-SHADOW_ORTHO, SHADOW_ORTHO,
            -SHADOW_ORTHO, SHADOW_ORTHO,
            SHADOW_NEAR, SHADOW_FAR);
        g_lightSpaceMatrix = lightProj * lightView;

        // Shadow Map
        glUseProgram(g_shadowShader);
        glUniformMatrix4fv(glGetUniformLocation(g_shadowShader, "lightSpaceMatrix"),
            1, GL_FALSE, glm::value_ptr(g_lightSpaceMatrix));

        glViewport(0, 0, SHADOW_W, SHADOW_H);
        glBindFramebuffer(GL_FRAMEBUFFER, g_shadowFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        glCullFace(GL_FRONT);

        // Ground
        {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(carPos.x, 0.0f, carPos.z));
            glUniformMatrix4fv(glGetUniformLocation(g_shadowShader, "model"), 1, GL_FALSE, glm::value_ptr(m));
            glBindVertexArray(VAO[1]);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }
        // Buildings
        for (auto& b : buildings) {
            if (!b.active) continue;
            float dx = b.pos.x - carPos.x, dz = b.pos.z - carPos.z;
            if (dx * dx + dz * dz > 120.0f * 120.0f) continue;
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(b.pos.x, b.height * 0.5f - 0.5f, b.pos.z));
            m = glm::scale(m, glm::vec3(2.5f, b.height, 2.5f));
            glUniformMatrix4fv(glGetUniformLocation(g_shadowShader, "model"), 1, GL_FALSE, glm::value_ptr(m));
            glBindVertexArray(VAO[0]);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
        // Obstacles obstacles shaded using different method as they are OBJ parser
        /*
        for (auto& obs : obstacles) {
            if (!obs.active) continue;
            glm::mat4 m = glm::translate(glm::mat4(1.0f), obs.pos + glm::vec3(0, obs.height * 0.5f, 0));
            m = glm::scale(m, glm::vec3(obs.width, obs.height, obs.depth));
            glUniformMatrix4fv(glGetUniformLocation(g_shadowShader, "model"), 1, GL_FALSE, glm::value_ptr(m));
            glBindVertexArray(VAO[0]);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }
        */
        
        // Car body
        // Car
        glm::mat4 carT = glm::translate(glm::mat4(1.0f), carPos);
        carT = glm::rotate(carT, glm::radians(carAngle), glm::vec3(0, 1, 0));

        auto drawShadowPart = [&](glm::vec3 pos, glm::vec3 scale) {
            glm::mat4 m = glm::translate(carT, pos);
            m = glm::scale(m, scale);

            glUniformMatrix4fv(
                glGetUniformLocation(g_shadowShader, "model"),
                1, GL_FALSE, glm::value_ptr(m)
            );

            glBindVertexArray(VAO[0]);
            glDrawArrays(GL_TRIANGLES, 0, 36);
            };

        // Body
        drawShadowPart({ 0.0f, 0.05f, 0.0f }, { 0.80f, 0.22f, 1.60f });

        // Roof
        drawShadowPart({ 0.0f, 0.30f, 0.05f }, { 0.55f, 0.20f, 0.80f });

        // Bumpers
        drawShadowPart({ 0.0f,-0.02f,-0.82f }, { 0.72f,0.12f,0.12f });

        // Wheels
        float wx[] = { 0.48f,-0.48f, 0.48f,-0.48f };
        float wz[] = { -0.55f,-0.55f,0.55f,0.55f };
        for (int i = 0; i < 4; ++i)
            drawShadowPart({ wx[i], -0.07f, wz[i] }, { 0.18f,0.28f,0.28f });
        // Trees
        for (auto& t : trees) {
            if (!t.active) continue;

            float dx = t.pos.x - carPos.x;
            float dz = t.pos.z - carPos.z;
            if (dx * dx + dz * dz > 130.0f * 130.0f) continue;

            glm::mat4 m = glm::translate(glm::mat4(1.0f),
                glm::vec3(t.pos.x, -0.5f, t.pos.z));

            m = glm::scale(m, glm::vec3(t.scale));

            glUniformMatrix4fv(glGetUniformLocation(g_shadowShader, "model"),
                1, GL_FALSE, glm::value_ptr(m));

            glBindVertexArray(VAO[0]);
            glDrawArrays(GL_TRIANGLES, 0, 36);
        }

        glCullFace(GL_BACK);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // Main render (Pass 2)
        glViewport(0, 0, WIN_W, WIN_H);

        if (g_crashed) {
            float t = g_crashTimer / 40.0f;
            glClearColor(0.8f * t + 0.53f * (1 - t), 0.65f * (1 - t * 0.4f), 0.82f * (1 - t), 1.0f);
        }
        else {
            glClearColor(0.53f, 0.65f, 0.82f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shader);
        glUniformMatrix4fv(glGetUniformLocation(shader, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shader, "projection"), 1, GL_FALSE, glm::value_ptr(proj));
        glUniformMatrix4fv(glGetUniformLocation(shader, "lightSpaceMatrix"), 1, GL_FALSE, glm::value_ptr(g_lightSpaceMatrix));
        glUniform3fv(glGetUniformLocation(shader, "uSunDir"), 1, glm::value_ptr(SUN_DIR));
        glUniform3fv(glGetUniformLocation(shader, "uSunColor"), 1, glm::value_ptr(SUN_COLOR));
        glUniform1f(glGetUniformLocation(shader, "uAmbient"), AMBIENT);

        
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_2D, g_shadowTex);
        glUniform1i(glGetUniformLocation(shader, "uShadowMap"), 1);

        // opaque pass
        glDepthMask(GL_TRUE);
        glUniform1f(uAlphaLoc, 1.0f);

        // Ground
        {
            glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(carPos.x, 0.0f, carPos.z));
            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(m));
            glUniform3f(glGetUniformLocation(shader, "uColor"), 0.18f, 0.18f, 0.20f);
            glBindVertexArray(VAO[1]);
            glDrawArrays(GL_TRIANGLES, 0, 6);
        }

        glUniform1i(glGetUniformLocation(shader, "uUseColor"), 1);

        // Smoothed road surface
        {
            const int SMOOTH_STEPS = 4;
            std::vector<RoadQuad> roadQuads;
            buildSmoothedRoadQuads(roadQuads, SMOOTH_STEPS);

            std::vector<float> triVerts;
            triVerts.reserve(roadQuads.size() * 6 * 3);
            for (auto& q : roadQuads) {
                triVerts.insert(triVerts.end(), { q.leftA.x,  q.leftA.y,  q.leftA.z });
                triVerts.insert(triVerts.end(), { q.rightA.x, q.rightA.y, q.rightA.z });
                triVerts.insert(triVerts.end(), { q.rightB.x, q.rightB.y, q.rightB.z });
                triVerts.insert(triVerts.end(), { q.leftA.x,  q.leftA.y,  q.leftA.z });
                triVerts.insert(triVerts.end(), { q.rightB.x, q.rightB.y, q.rightB.z });
                triVerts.insert(triVerts.end(), { q.leftB.x,  q.leftB.y,  q.leftB.z });
            }
            glBindVertexArray(dynVAO);
            glBindBuffer(GL_ARRAY_BUFFER, dynVBO);
            glBufferData(GL_ARRAY_BUFFER, triVerts.size() * sizeof(float), triVerts.data(), GL_DYNAMIC_DRAW);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
            glEnableVertexAttribArray(0);
            glDisableVertexAttribArray(1);

            glm::mat4 identity(1.0f);
            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(identity));
            glUniform3f(glGetUniformLocation(shader, "uColor"), 0.10f, 0.10f, 0.12f);
            glUniform1i(glGetUniformLocation(shader, "uUseColor"), 1);
            glDrawArrays(GL_TRIANGLES, 0, (GLsizei)(triVerts.size() / 3));

            glBindVertexArray(VAO[0]);
            glEnableVertexAttribArray(1);
        }

        // Centre dashed line
        {
            const int SMOOTH_STEPS = 4;
            int n = (int)roadSegs.size();
            glm::mat4 identity(1.0f);
            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(identity));
            glUniform3f(glGetUniformLocation(shader, "uColor"), 0.90f, 0.85f, 0.20f);
            glUniform1i(glGetUniformLocation(shader, "uUseColor"), 1);

            for (int i = 1; i + 2 < n; i += 4) {
                std::vector<float> dashPts;
                glm::vec2 p0 = roadSegs[i - 1].center, p1 = roadSegs[i].center;
                glm::vec2 p2 = (i + 1 < n) ? roadSegs[i + 1].center : p1;
                glm::vec2 p3 = (i + 2 < n) ? roadSegs[i + 2].center : p2;
                for (int s = 0; s <= SMOOTH_STEPS; ++s) {
                    float t = float(s) / float(SMOOTH_STEPS);
                    glm::vec2 c = catmullRom(p0, p1, p2, p3, t);
                    dashPts.insert(dashPts.end(), { c.x, -0.480f, c.y });
                }
                glBindVertexArray(lineVAO);
                glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
                glBufferData(GL_ARRAY_BUFFER, dashPts.size() * sizeof(float), dashPts.data(), GL_DYNAMIC_DRAW);
                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
                glEnableVertexAttribArray(0);
                glDisableVertexAttribArray(1);
                glLineWidth(3.5f);
                glDrawArrays(GL_LINE_STRIP, 0, (GLsizei)(dashPts.size() / 3));
            }
        }

        // Road barriers
        {
            const int SMOOTH_STEPS = 4;
            std::vector<glm::vec3> leftBarrier, rightBarrier;
            buildSmoothedBarriers(leftBarrier, rightBarrier, SMOOTH_STEPS);
            glBindVertexArray(lineVAO);
            glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
            glm::mat4 identity(1.0f);
            glUniformMatrix4fv(glGetUniformLocation(shader, "model"), 1, GL_FALSE, glm::value_ptr(identity));
            glUniform3f(glGetUniformLocation(shader, "uColor"), 1.0f, 1.0f, 1.0f);
            glUniform1i(glGetUniformLocation(shader, "uUseColor"), 1);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
            glEnableVertexAttribArray(0);
            glDisableVertexAttribArray(1);
            glLineWidth(3.5f);
            glBufferData(GL_ARRAY_BUFFER, leftBarrier.size() * sizeof(glm::vec3), leftBarrier.data(), GL_DYNAMIC_DRAW);
            glDrawArrays(GL_LINE_STRIP, 0, (GLsizei)leftBarrier.size());
            glBufferData(GL_ARRAY_BUFFER, rightBarrier.size() * sizeof(glm::vec3), rightBarrier.data(), GL_DYNAMIC_DRAW);
            glDrawArrays(GL_LINE_STRIP, 0, (GLsizei)rightBarrier.size());
            glLineWidth(1.0f);
            glBindVertexArray(VAO[0]);
            glEnableVertexAttribArray(1);
        }

        if (!isCarOnRoad(carPos) || g_lives <= 0) resetGame();

        // Buildings
        for (auto& b : buildings) {
            if (!b.active) continue;
            float dx = b.pos.x - carPos.x, dz = b.pos.z - carPos.z;
            if (dx * dx + dz * dz > 120.0f * 120.0f) continue;
            drawBox(shader, VAO[0], glm::vec3(b.pos.x, b.height * 0.5f - 0.5f, b.pos.z), glm::vec3(2.5f, b.height, 2.5f), b.r, b.g, b.b);
            drawBox(shader, VAO[0], glm::vec3(b.pos.x, b.height - 0.5f + 0.15f, b.pos.z), glm::vec3(2.6f, 0.25f, 2.6f),
                fminf(b.r + 0.12f, 1), fminf(b.g + 0.12f, 1), fminf(b.b + 0.12f, 1));
            if ((abs((int)(b.pos.x + b.pos.z)) % 4) == 0)
                drawBox(shader, VAO[0], glm::vec3(b.pos.x, b.height + 0.2f, b.pos.z), glm::vec3(0.20f, 0.60f, 0.20f), 0.22f, 0.22f, 0.25f);
        }

        // Trees
        for (auto& t : trees) {
            if (!t.active) continue;
            float dx = t.pos.x - carPos.x, dz = t.pos.z - carPos.z;
            if (dx * dx + dz * dz > 130.0f * 130.0f) continue;
            drawTree(shader, VAO[0], t);
        }

        // Obstacles
        for (auto& obs : obstacles) {
            if (!obs.active) continue;
            float dx = obs.pos.x - carPos.x, dz = obs.pos.z - carPos.z;
            if (dx * dx + dz * dz > 80.0f * 80.0f) continue;
            if (obs.type == 0 && g_obs1Mesh.VAO != 0) {
                glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(obs.pos.x, obs.pos.y + 0.3f, obs.pos.z));
                float sf = 0.03f;
                m = glm::scale(m, glm::vec3(obs.width * sf * 1.5f, obs.height * sf, obs.depth * sf));
                drawOBJ(shader, g_obs1Mesh, m, 0);
            }
            else if (obs.type == 1 && g_obs2Mesh.VAO != 0) {
                glm::mat4 m = glm::translate(glm::mat4(1.0f), glm::vec3(obs.pos.x, obs.pos.y + 0.1f, obs.pos.z));
                m = glm::rotate(m, glm::radians(90.0f), glm::vec3(0, 1, 0));
                float sf = obs.width / 2.0f;
                m = glm::scale(m, glm::vec3(sf));
                glUniform3f(glGetUniformLocation(shader, "uColor"), g_obs2Mesh.material.Kd.r, g_obs2Mesh.material.Kd.g, g_obs2Mesh.material.Kd.b);
                glUniform1i(glGetUniformLocation(shader, "uUseVertexColor"), 0);
                drawOBJ(shader, g_obs2Mesh, m, 1);
            }
        }

        if (!isCarOnRoad(carPos) || g_lives <= 0) resetGame();

        // ---- Car ----
        {
            glm::mat4 carT = glm::translate(glm::mat4(1.0f), carPos);
            carT = glm::rotate(carT, glm::radians(carAngle), glm::vec3(0, 1, 0));

            float cr = 0.08f, cg = 0.12f, cb = 0.32f;
            if (g_crashed && (int)(g_crashTimer) % 8 < 4) { cr = 1.0f; cg = 1.0f; cb = 1.0f; }

            drawCarPart(shader, VAO[0], carT, { 0.0f, 0.05f, 0.0f }, { 0.80f,0.22f,1.60f }, cr, cg, cb);
            drawCarPart(shader, VAO[0], carT, { 0.0f, 0.30f, 0.05f }, { 0.55f,0.20f,0.80f }, 0.55f, 0.75f, 0.95f, 0.35f);
            drawCarPart(shader, VAO[0], carT, { 0.0f,-0.02f,-0.82f }, { 0.72f,0.12f,0.12f }, 0.20f, 0.25f, 0.40f);
            drawCarPart(shader, VAO[0], carT, { 0.25f,0.02f,-0.85f }, { 0.12f,0.08f,0.06f }, 0.95f, 0.10f, 0.10f);
            drawCarPart(shader, VAO[0], carT, { -0.25f,0.02f,-0.85f }, { 0.12f,0.08f,0.06f }, 0.95f, 0.10f, 0.10f);
            drawCarPart(shader, VAO[0], carT, { 0.25f,0.02f, 0.82f }, { 0.10f,0.08f,0.06f }, 0.85f, 0.05f, 0.05f);
            drawCarPart(shader, VAO[0], carT, { -0.25f,0.02f, 0.82f }, { 0.10f,0.08f,0.06f }, 0.85f, 0.05f, 0.05f);
            float wx[] = { 0.48f,-0.48f, 0.48f,-0.48f };
            float wz[] = { -0.55f,-0.55f,0.55f,0.55f };
            for (int i = 0; i < 4; ++i)
                drawCarPart(shader, VAO[0], carT, { wx[i],-0.07f,wz[i] }, { 0.18f,0.28f,0.28f }, 0.10f, 0.10f, 0.10f);
        }

        // Clouds (Transparent)
        glDepthMask(GL_FALSE);

        std::sort(clouds.begin(), clouds.end(), [&](const Cloud& a, const Cloud& b_) {
            return glm::distance(camPos, a.pos) > glm::distance(camPos, b_.pos);
            });

        glUniform1i(glGetUniformLocation(shader, "uUseColor"), 1);
        glUniform1i(glGetUniformLocation(shader, "uUseVertexColor"), 0);
        glBindVertexArray(VAO[0]);
        glEnableVertexAttribArray(1);

        for (auto& c : clouds) {
            float dx = c.pos.x - carPos.x, dz = c.pos.z - carPos.z;
            if (dx * dx + dz * dz > 200.0f * 200.0f) continue;
            drawCloud(shader, VAO[0], c, uAlphaLoc);
        }

        glDepthMask(GL_TRUE);

        std::cout << "\rScore: " << g_score << "  High: " << g_highScore << std::flush;

        glfwSwapBuffers(win);
        glfwPollEvents();
    }

    glDeleteVertexArrays(1, &dynVAO);
    glDeleteBuffers(1, &dynVBO);
    glDeleteVertexArrays(1, &lineVAO);
    glDeleteBuffers(1, &lineVBO);
    FreeOBJ(g_obs1Mesh);
    FreeOBJ(g_obs2Mesh);
    glfwTerminate();
    return 0;
}