#pragma once
// ============================================================
//  OBJLoader.h  –  Wavefront OBJ + MTL parser
//  Supports: v, vn, vt, f, mtllib, usemtl
//  Produces interleaved vertex buffer + separate colour buffer.
//
//  Vertex buffer layout (stride = 8 floats):
//    [0..2]  position  (x, y, z)
//    [3..5]  normal    (nx, ny, nz)
//    [6..7]  texcoord  (u, v)
//
//  Colour buffer layout (stride = 3 floats):
//    [0..2]  diffuse rgb from MTL Kd, one entry per vertex
//
//  GL attrib slots after UploadOBJ:
//    0 = position, 1 = normal, 2 = uv, 3 = colour
// ============================================================

#include <string>
#include <vector>
#include <map>
#include <fstream>
#include <sstream>
#include <iostream>
#include <array>

struct Material {
    glm::vec3 Ka = glm::vec3(1.0f);
    glm::vec3 Kd = glm::vec3(1.0f);
    glm::vec3 Ke = glm::vec3(0.0f);
};

struct OBJMesh {
    std::vector<float> vertices;
    std::vector<float> colors;

    unsigned int VAO = 0;
    unsigned int VBO = 0;
    unsigned int colorVBO = 0;

    int vertexCount = 0;

    Material material;   // ? ADD THIS (important for obstacle2)
};


// -------------------------------------------------------
// LoadMTL  –  parse a .mtl file, return map name->Material
// -------------------------------------------------------
inline std::map<std::string, Material> LoadMTL(const std::string& path)
{
    std::map<std::string, Material> mats;

    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "[OBJLoader] MTL not found: " << path << "\n";
        return mats;
    }

    std::string line, curName;

    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string tok;
        ss >> tok;

        if (tok == "newmtl") {
            ss >> curName;
            mats[curName] = Material();
        }

        else if (tok == "Ka" && !curName.empty()) {
            ss >> mats[curName].Ka.r >> mats[curName].Ka.g >> mats[curName].Ka.b;
        }

        else if (tok == "Kd" && !curName.empty()) {
            ss >> mats[curName].Kd.r >> mats[curName].Kd.g >> mats[curName].Kd.b;
        }

        else if (tok == "Ke" && !curName.empty()) {
            ss >> mats[curName].Ke.r >> mats[curName].Ke.g >> mats[curName].Ke.b;
        }
    }

    std::cout << "[OBJLoader] Loaded MTL: " << path
        << "  ->  " << mats.size() << " materials\n";

    return mats;
}


// -------------------------------------------------------
// Internal helper: parse face token "v", "v/t", "v//n", "v/t/n"
// -------------------------------------------------------
static void parseFaceToken(const std::string& tok, int& vi, int& ti, int& ni)
{
    vi = ti = ni = 0;
    std::istringstream ss(tok);
    std::string part;
    int idx = 0;
    while (std::getline(ss, part, '/')) {
        if (!part.empty()) {
            int val = std::stoi(part);
            if (idx == 0) vi = val;
            else if (idx == 1) ti = val;
            else if (idx == 2) ni = val;
        }
        ++idx;
    }
}

// -------------------------------------------------------
// Helper: extract directory from a file path
// -------------------------------------------------------
static std::string dirOf(const std::string& path)
{
    size_t pos = path.find_last_of("/\\");
    return (pos == std::string::npos) ? "" : path.substr(0, pos + 1);
}

// -------------------------------------------------------
// LoadOBJ  –  returns false on failure
// -------------------------------------------------------
inline bool LoadOBJ(const std::string& path, OBJMesh& mesh)
{
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[OBJLoader] Cannot open: " << path << "\n";
        return false;
    }

    std::vector<std::array<float, 3>> positions;
    std::vector<std::array<float, 3>> normals;
    std::vector<std::array<float, 2>> texcoords;

    const std::array<float, 3> defaultNormal = { 0.f, 1.f, 0.f };

    // MTL state
    std::map<std::string, Material> materials;
    Material curMat;   // default: 0.8 grey

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string token;
        ss >> token;

        if (token == "v") {
            float x, y, z; ss >> x >> y >> z;
            positions.push_back({ x, y, z });
        }
        else if (token == "vn") {
            float x, y, z; ss >> x >> y >> z;
            normals.push_back({ x, y, z });
        }
        else if (token == "vt") {
            float u, v = 0.f; ss >> u >> v;
            texcoords.push_back({ u, v });
        }
        else if (token == "mtllib") {
            std::string mtlFile; ss >> mtlFile;
            materials = LoadMTL(dirOf(path) + mtlFile);
        }
        else if (token == "usemtl") {
            std::string matName; ss >> matName;

            auto it = materials.find(matName);
            if (it != materials.end()) {
                curMat = it->second;
            }
            else {
                curMat = Material{};
            }

            // ? STORE INTO MESH (this is what obstacle2 will use later)
            mesh.material = curMat;
        }
        else if (token == "f") {
            std::vector<int> vis, tis, nis;
            std::string tok;
            while (ss >> tok) {
                int vi, ti, ni;
                parseFaceToken(tok, vi, ti, ni);
                vis.push_back(vi);
                tis.push_back(ti);
                nis.push_back(ni);
            }

            // Fan triangulation
            for (int i = 1; i + 1 < (int)vis.size(); ++i) {
                int fan[3] = { 0, i, i + 1 };
                for (int f : fan) {
                    // Position
                    int pi = vis[f] > 0 ? vis[f] - 1 : (int)positions.size() + vis[f];
                    auto& p = positions[pi];
                    mesh.vertices.push_back(p[0]);
                    mesh.vertices.push_back(p[1]);
                    mesh.vertices.push_back(p[2]);

                    // Normal
                    if (nis[f] != 0 && !normals.empty()) {
                        int ni2 = nis[f] > 0 ? nis[f] - 1 : (int)normals.size() + nis[f];
                        auto& n = normals[ni2];
                        mesh.vertices.push_back(n[0]);
                        mesh.vertices.push_back(n[1]);
                        mesh.vertices.push_back(n[2]);
                    }
                    else {
                        mesh.vertices.push_back(defaultNormal[0]);
                        mesh.vertices.push_back(defaultNormal[1]);
                        mesh.vertices.push_back(defaultNormal[2]);
                    }

                    // UV
                    if (tis[f] != 0 && !texcoords.empty()) {
                        int ti2 = tis[f] > 0 ? tis[f] - 1 : (int)texcoords.size() + tis[f];
                        mesh.vertices.push_back(texcoords[ti2][0]);
                        mesh.vertices.push_back(texcoords[ti2][1]);
                    }
                    else {
                        mesh.vertices.push_back(0.f);
                        mesh.vertices.push_back(0.f);
                    }

                    // Colour from current material
                    mesh.colors.push_back(curMat.Kd.r);
                    mesh.colors.push_back(curMat.Kd.g);
                    mesh.colors.push_back(curMat.Kd.b);

                }
            }
        }
        // skip o, g, s, usemtl without mtl loaded, etc.
    }

    mesh.vertexCount = (int)mesh.vertices.size() / 8;
    std::cout << "[OBJLoader] Loaded " << path
        << "  ->  " << mesh.vertexCount << " vertices\n";
    return mesh.vertexCount > 0;
}

// -------------------------------------------------------
// UploadOBJ  –  creates VAO/VBOs and uploads to GPU
//   Attrib 0: position (vec3)
//   Attrib 1: normal   (vec3)
//   Attrib 2: texcoord (vec2)
//   Attrib 3: colour   (vec3)  <-- from MTL Kd
// -------------------------------------------------------
#include <GL/gl3w.h>

inline bool UploadOBJ(OBJMesh& mesh)
{
    if (mesh.vertices.empty()) return false;

    glGenVertexArrays(1, &mesh.VAO);
    glBindVertexArray(mesh.VAO);

    // --- Geometry VBO (pos + normal + uv) ---
    glGenBuffers(1, &mesh.VBO);
    glBindBuffer(GL_ARRAY_BUFFER, mesh.VBO);
    glBufferData(GL_ARRAY_BUFFER,
        mesh.vertices.size() * sizeof(float),
        mesh.vertices.data(),
        GL_STATIC_DRAW);

    const GLsizei stride = 8 * sizeof(float);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // --- Colour VBO (rgb per vertex) ---
    if (!mesh.colors.empty()) {
        glGenBuffers(1, &mesh.colorVBO);
        glBindBuffer(GL_ARRAY_BUFFER, mesh.colorVBO);
        glBufferData(GL_ARRAY_BUFFER,
            mesh.colors.size() * sizeof(float),
            mesh.colors.data(),
            GL_STATIC_DRAW);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(3);
    }

    glBindVertexArray(0);
    return true;
}

// -------------------------------------------------------
// FreeOBJ  –  cleans up GPU resources
// -------------------------------------------------------
inline void FreeOBJ(OBJMesh& mesh)
{
    if (mesh.colorVBO) glDeleteBuffers(1, &mesh.colorVBO);
    if (mesh.VBO)      glDeleteBuffers(1, &mesh.VBO);
    if (mesh.VAO)      glDeleteVertexArrays(1, &mesh.VAO);
    mesh.VAO = mesh.VBO = mesh.colorVBO = 0;
}