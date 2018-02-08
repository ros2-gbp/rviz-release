/*
 * Copyright (c) 2010, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mesh_loader.hpp"

#include <map>
#include <string>
#include <vector>

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include <OgreHardwareBufferManager.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgreMeshManager.h>
#include <OgreMeshSerializer.h>
#include <OgrePass.h>
#include <OgreSharedPtr.h>
#include <OgreSubMesh.h>
#include <OgreTechnique.h>
#include <OgreTechnique.h>
#include <OgreTexture.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>

#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include <QDir>
#include <QFileInfo>
#include <QString>

// #include <tinyxml.h>

#define ASSIMP_UNIFIED_HEADER_NAMES 1
#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif

#include "resource_retriever/retriever.h"
#include "./stl_loader.hpp"

#include "rviz_common/logging.hpp"

#define ROS_PACKAGE_NAME "rviz_common"

namespace rviz
{

class ResourceIOStream : public Assimp::IOStream
{
public:
  explicit ResourceIOStream(const resource_retriever::MemoryResource & res)
  : res_(res),
    pos_(res.data.get())
  {}

  ~ResourceIOStream()
  {}

  size_t Read(void * buffer, size_t size, size_t count)
  {
    size_t to_read = size * count;
    if (pos_ + to_read > res_.data.get() + res_.size) {
      to_read = res_.size - (pos_ - res_.data.get());
    }

    memcpy(buffer, pos_, to_read);
    pos_ += to_read;

    return to_read;
  }

  size_t Write(const void * buffer, size_t size, size_t count)
  {
    (void) buffer;
    (void) size;
    (void) count;
    throw std::runtime_error("expected to be overridden");
  }

  aiReturn Seek(size_t offset, aiOrigin origin)
  {
    uint8_t * new_pos = 0;
    switch (origin) {
      case aiOrigin_SET:
        new_pos = res_.data.get() + offset;
        break;
      case aiOrigin_CUR:
        new_pos = pos_ + offset;  // TODO(anyone): is this right? Can offset really not be negative
        break;
      case aiOrigin_END:
        new_pos = res_.data.get() + res_.size - offset;  // TODO(anyone): is this right?
        break;
      default:
        throw std::runtime_error("unexpected default in switch statement");
    }

    if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size) {
      return aiReturn_FAILURE;
    }

    pos_ = new_pos;
    return aiReturn_SUCCESS;
  }

  size_t Tell() const
  {
    return pos_ - res_.data.get();
  }

  size_t FileSize() const
  {
    return res_.size;
  }

  void Flush() {}

private:
  resource_retriever::MemoryResource res_;
  uint8_t * pos_;
};

class ResourceIOSystem : public Assimp::IOSystem
{
public:
  ResourceIOSystem()
  {
  }

  ~ResourceIOSystem()
  {
  }

  // Check whether a specific file exists
  bool Exists(const char * file) const
  {
    // Ugly -- two retrievals where there should be one (Exists + Open)
    // resource_retriever needs a way of checking for existence
    // TODO(anyone): cache this
    resource_retriever::MemoryResource res;
    try {
      res = retriever_.get(file);
    } catch (resource_retriever::Exception & e) {
      (void) e;  // do nothing on exception
      return false;
    }

    return true;
  }

  // Get the path delimiter character we'd like to see
  char getOsSeparator() const
  {
    return '/';
  }

  // ... and finally a method to open a custom stream
  Assimp::IOStream * Open(const char * file, const char * mode = "rb")
  {
    (void) mode;
    assert(mode == std::string("r") || mode == std::string("rb"));

    // Ugly -- two retrievals where there should be one (Exists + Open)
    // resource_retriever needs a way of checking for existence
    resource_retriever::MemoryResource res;
    try {
      res = retriever_.get(file);
    } catch (resource_retriever::Exception & e) {
      (void) e;  // do nothing on exception
      return 0;
    }

    return new ResourceIOStream(res);
  }

  void Close(Assimp::IOStream * stream);

private:
  mutable resource_retriever::Retriever retriever_;
};

void ResourceIOSystem::Close(Assimp::IOStream * stream)
{
  delete stream;
}

// Mostly stolen from gazebo
/** @brief Recursive mesh-building function.
 * @param scene is the assimp scene containing the whole mesh.
 * @param node is the current assimp node, which is part of a tree of nodes being recursed over.
 * @param material_table is indexed the same as scene->mMaterials[], and should have been filled out already by loadMaterials(). */
void buildMesh(
  const aiScene * scene, const aiNode * node,
  const Ogre::MeshPtr & mesh,
  Ogre::AxisAlignedBox & aabb, float & radius, const float scale,
  std::vector<Ogre::MaterialPtr> & material_table)
{
  if (!node) {
    return;
  }

  aiMatrix4x4 transform = node->mTransformation;
  aiNode * pnode = node->mParent;
  while (pnode) {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL) {
      transform = pnode->mTransformation * transform;
    }
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();

  for (uint32_t i = 0; i < node->mNumMeshes; i++) {
    aiMesh * input_mesh = scene->mMeshes[node->mMeshes[i]];

    Ogre::SubMesh * submesh = mesh->createSubMesh();
    submesh->useSharedVertices = false;
    submesh->vertexData = new Ogre::VertexData();
    Ogre::VertexData * vertex_data = submesh->vertexData;
    Ogre::VertexDeclaration * vertex_decl = vertex_data->vertexDeclaration;

    size_t offset = 0;
    // positions
    vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
    offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

    // normals
    if (input_mesh->HasNormals()) {
      vertex_decl->addElement(0, offset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
    }

    // texture coordinates (only support 1 for now)
    if (input_mesh->HasTextureCoords(0)) {
      vertex_decl->addElement(0, offset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
      offset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);
    }

    // TODO(anyone): vertex colors

    // allocate the vertex buffer
    vertex_data->vertexCount = input_mesh->mNumVertices;
    Ogre::HardwareVertexBufferSharedPtr vbuf =
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(vertex_decl->getVertexSize(
          0),
        vertex_data->vertexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

    vertex_data->vertexBufferBinding->setBinding(0, vbuf);
    float * vertices = static_cast<float *>(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumVertices; j++) {
      aiVector3D p = input_mesh->mVertices[j];
      p *= transform;
      p *= scale;
      *vertices++ = p.x;
      *vertices++ = p.y;
      *vertices++ = p.z;

      Ogre::Vector3 v(p.x, p.y, p.z);
      aabb.merge(v);
      float dist = v.length();
      if (dist > radius) {
        radius = dist;
      }

      if (input_mesh->HasNormals()) {
        aiVector3D n = inverse_transpose_rotation * input_mesh->mNormals[j];
        n.Normalize();
        *vertices++ = n.x;
        *vertices++ = n.y;
        *vertices++ = n.z;
      }

      if (input_mesh->HasTextureCoords(0)) {
        *vertices++ = input_mesh->mTextureCoords[0][j].x;
        *vertices++ = input_mesh->mTextureCoords[0][j].y;
      }
    }

    // calculate index count
    submesh->indexData->indexCount = 0;
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
      aiFace & face = input_mesh->mFaces[j];
      submesh->indexData->indexCount += face.mNumIndices;
    }

    // If we have less than 65536 (2^16) vertices, we can use a 16-bit index buffer.
    if (vertex_data->vertexCount < (1 << 16) ) {
      // allocate index buffer
      submesh->indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_16BIT,
        submesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

      Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
      uint16_t * indices = static_cast<uint16_t *>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      // add the indices
      for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
        aiFace & face = input_mesh->mFaces[j];
        for (uint32_t k = 0; k < face.mNumIndices; ++k) {
          *indices++ = face.mIndices[k];
        }
      }

      ibuf->unlock();
    } else {
      // Else we have more than 65536 (2^16) vertices, so we must
      // use a 32-bit index buffer (or subdivide the mesh, which
      // I'm too impatient to do right now)

      // allocate index buffer
      submesh->indexData->indexBuffer =
        Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
        Ogre::HardwareIndexBuffer::IT_32BIT,
        submesh->indexData->indexCount,
        Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY,
        false);

      Ogre::HardwareIndexBufferSharedPtr ibuf = submesh->indexData->indexBuffer;
      uint32_t * indices = static_cast<uint32_t *>(ibuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));

      // add the indices
      for (uint32_t j = 0; j < input_mesh->mNumFaces; j++) {
        aiFace & face = input_mesh->mFaces[j];
        for (uint32_t k = 0; k < face.mNumIndices; ++k) {
          *indices++ = face.mIndices[k];
        }
      }

      ibuf->unlock();
    }
    vbuf->unlock();

    submesh->setMaterialName(material_table[input_mesh->mMaterialIndex]->getName());
  }

  for (uint32_t i = 0; i < node->mNumChildren; ++i) {
    buildMesh(scene, node->mChildren[i], mesh, aabb, radius, scale, material_table);
  }
}

void loadTexture(const std::string & resource_path)
{
  if (!Ogre::TextureManager::getSingleton().resourceExists(resource_path, "rviz_common")) {
    resource_retriever::Retriever retriever;
    resource_retriever::MemoryResource res;
    try {
      res = retriever.get(resource_path);
    } catch (resource_retriever::Exception & e) {
      fprintf(stderr, "%s", e.what());
    }

    if (res.size != 0) {
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::Image image;
      QFileInfo resource_path_finfo(QString::fromStdString(resource_path));
      std::string extension = resource_path_finfo.completeSuffix().toStdString();

      if (extension[0] == '.') {
        extension = extension.substr(1, extension.size() - 1);
      }

      try {
        image.load(stream, extension);
        Ogre::TextureManager::getSingleton().loadImage(resource_path, "rviz_common", image);
      } catch (Ogre::Exception & e) {
        fprintf(stderr, "Could not load texture [%s]: %s", resource_path.c_str(), e.what());
      }
    }
  }
}

// Mostly cribbed from gazebo
/** @brief Load all materials needed by the given scene.
 * @param resource_path the path to the resource from which this scene is being loaded.
 *        loadMaterials() assumes textures for this scene are relative to the same directory that this scene is in.
 * @param scene the assimp scene to load materials for.
 * @param material_table_out Reference to the resultant material table, filled out by this function.  Is indexed the same as scene->mMaterials[].
 */
void loadMaterials(
  const std::string & resource_path,
  const aiScene * scene,
  std::vector<Ogre::MaterialPtr> & material_table_out)
{
  for (uint32_t i = 0; i < scene->mNumMaterials; i++) {
    std::stringstream ss;
    ss << resource_path << "Material" << i;
    Ogre::MaterialPtr mat = Ogre::MaterialManager::getSingleton().create(
      ss.str(), ROS_PACKAGE_NAME, true);
    material_table_out.push_back(mat);

    Ogre::Technique * tech = mat->getTechnique(0);
    Ogre::Pass * pass = tech->getPass(0);

    aiMaterial * amat = scene->mMaterials[i];

    Ogre::ColourValue diffuse(1.0, 1.0, 1.0, 1.0);
    Ogre::ColourValue specular(1.0, 1.0, 1.0, 1.0);
    Ogre::ColourValue ambient(0, 0, 0, 1.0);

    for (uint32_t j = 0; j < amat->mNumProperties; j++) {
      aiMaterialProperty * prop = amat->mProperties[j];
      std::string propKey = prop->mKey.data;

      if (propKey == "$tex.file") {
        aiString texName;
        aiTextureMapping mapping;
        uint32_t uvIndex;
        amat->GetTexture(aiTextureType_DIFFUSE, 0, &texName, &mapping, &uvIndex);

        // Assume textures are in paths relative to the mesh
        QFileInfo resource_path_finfo(QString::fromStdString(resource_path));
        QDir resource_path_qdir = resource_path_finfo.dir();
        QString foo = resource_path_qdir.path().replace("package://hsr_meshes",
            "file:///Users/william/hsr_ws/src/hsr_meshes");
        std::string texture_path = foo.toStdString() + "/" + texName.data;
        printf("asdf: %s\n", foo.toStdString().c_str());
        loadTexture(texture_path);
        Ogre::TextureUnitState * tu = pass->createTextureUnitState();
        tu->setTextureName(texture_path);
      } else if (propKey == "$clr.diffuse") {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);
        diffuse = Ogre::ColourValue(clr.r, clr.g, clr.b);
      } else if (propKey == "$clr.ambient") {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_AMBIENT, clr);
        ambient = Ogre::ColourValue(clr.r, clr.g, clr.b);
      } else if (propKey == "$clr.specular") {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_SPECULAR, clr);
        specular = Ogre::ColourValue(clr.r, clr.g, clr.b);
      } else if (propKey == "$clr.emissive") {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_EMISSIVE, clr);
        mat->setSelfIllumination(clr.r, clr.g, clr.b);
      } else if (propKey == "$clr.opacity") {
        float o;
        amat->Get(AI_MATKEY_OPACITY, o);
        diffuse.a = o;
      } else if (propKey == "$mat.shininess") {
        float s;
        amat->Get(AI_MATKEY_SHININESS, s);
        mat->setShininess(s);
      } else if (propKey == "$mat.shadingm") {
        int model;
        amat->Get(AI_MATKEY_SHADING_MODEL, model);
        switch (model) {
          case aiShadingMode_Flat:
            mat->setShadingMode(Ogre::SO_FLAT);
            break;
          case aiShadingMode_Phong:
            mat->setShadingMode(Ogre::SO_PHONG);
            break;
          case aiShadingMode_Gouraud:
          default:
            mat->setShadingMode(Ogre::SO_GOURAUD);
            break;
        }
      }
    }

    int mode = aiBlendMode_Default;
    amat->Get(AI_MATKEY_BLEND_FUNC, mode);
    switch (mode) {
      case aiBlendMode_Additive:
        mat->setSceneBlending(Ogre::SBT_ADD);
        break;
      case aiBlendMode_Default:
      default:
        {
          if (diffuse.a < 0.99) {
            pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
          } else {
            pass->setSceneBlending(Ogre::SBT_REPLACE);
          }
        }
        break;
    }

    mat->setAmbient(ambient * 0.5);
    mat->setDiffuse(diffuse);
    specular.a = diffuse.a;
    mat->setSpecular(specular);
  }
}


/*@brief - Get the scaling from units used in this mesh file to meters.

  This function applies only to Collada files. It is necessary because
  ASSIMP does not currently expose an api to retrieve the scaling factor.

  @Param[in] resource_path   -   The url of a resource containing a mesh.

  @Returns The scaling factor that converts the mesh to meters. Returns 1.0
  for meshes which do not explicitly encode such a scaling.

*/

float getMeshUnitRescale(const std::string & resource_path)
{
#if 0
  static std::map<std::string, float> rescale_cache;


  // Try to read unit to meter conversion ratio from mesh. Only valid in Collada XML formats.
  TiXmlDocument xmlDoc;
  float unit_scale(1.0);
  resource_retriever::Retriever retriever;
  resource_retriever::MemoryResource res;
  try {
    res = retriever.get(resource_path);
  } catch (resource_retriever::Exception & e) {
    fprintf(stderr, "%s", e.what());
    return unit_scale;
  }

  if (res.size == 0) {
    return unit_scale;
  }


  // Use the resource retriever to get the data.
  const char * data = reinterpret_cast<const char *>(res.data.get());
  xmlDoc.Parse(data);

  // Find the appropriate element if it exists
  if (!xmlDoc.Error()) {
    TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
    if (colladaXml) {
      TiXmlElement * assetXml = colladaXml->FirstChildElement("asset");
      if (assetXml) {
        TiXmlElement * unitXml = assetXml->FirstChildElement("unit");
        if (unitXml && unitXml->Attribute("meter")) {
          // Failing to convert leaves unit_scale as the default.
          if (unitXml->QueryFloatAttribute("meter", &unit_scale) != 0) {
            RVIZ_COMMON_LOG_WARNING_STREAM(
              "getMeshUnitRescale::Failed to convert unit element meter "
              "attribute to determine scaling. unit element: " << *unitXml);
          }
        }
      }
    }
  }
  return unit_scale;
#endif
  (void) resource_path;
  return 1;
}


Ogre::MeshPtr meshFromAssimpScene(const std::string & name, const aiScene * scene)
{
  if (!scene->HasMeshes()) {
    fprintf(stderr, "No meshes found in file [%s]", name.c_str());
    return Ogre::MeshPtr();
  }

  std::vector<Ogre::MaterialPtr> material_table;
  loadMaterials(name, scene, material_table);

  Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(name, ROS_PACKAGE_NAME);

  Ogre::AxisAlignedBox aabb(Ogre::AxisAlignedBox::EXTENT_NULL);
  float radius = 0.0f;
  float scale = getMeshUnitRescale(name);
  buildMesh(scene, scene->mRootNode, mesh, aabb, radius, scale, material_table);

  mesh->_setBounds(aabb);
  mesh->_setBoundingSphereRadius(radius);
  mesh->buildEdgeList();

  mesh->load();

  return mesh;
}

Ogre::MeshPtr loadMeshFromResource(const std::string & resource_path)
{
  if (Ogre::MeshManager::getSingleton().resourceExists(resource_path, ROS_PACKAGE_NAME)) {
    return Ogre::MeshManager::getSingleton().getByName(resource_path, ROS_PACKAGE_NAME);
  } else {
    QFileInfo model_path(QString::fromStdString(resource_path));
    std::string ext = model_path.completeSuffix().toStdString();
    if (ext == ".mesh" || ext == ".MESH") {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try {
        res = retriever.get(resource_path);
      } catch (resource_retriever::Exception & e) {
        fprintf(stderr, "%s", e.what());
        return Ogre::MeshPtr();
      }

      if (res.size == 0) {
        return Ogre::MeshPtr();
      }

      Ogre::MeshSerializer ser;
      Ogre::DataStreamPtr stream(new Ogre::MemoryDataStream(res.data.get(), res.size));
      Ogre::MeshPtr mesh = Ogre::MeshManager::getSingleton().createManual(resource_path, "rviz");
      ser.importMesh(stream, mesh.get());

      return mesh;
    } else if (ext == ".stl" || ext == ".STL" || ext == ".stlb" || ext == ".STLB") {
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try {
        res = retriever.get(resource_path);
      } catch (resource_retriever::Exception & e) {
        fprintf(stderr, "%s", e.what());
        return Ogre::MeshPtr();
      }

      if (res.size == 0) {
        return Ogre::MeshPtr();
      }

      ogre_tools::STLLoader loader;
      if (!loader.load(res.data.get(), res.size, resource_path)) {
        fprintf(stderr, "Failed to load file [%s]", resource_path.c_str());
        return Ogre::MeshPtr();
      }

      return loader.toMesh(resource_path);
    } else {
      Assimp::Importer importer;
      importer.SetIOHandler(new ResourceIOSystem());
      const aiScene * scene = importer.ReadFile(resource_path,
          aiProcess_SortByPType | aiProcess_GenNormals | aiProcess_Triangulate |
          aiProcess_GenUVCoords | aiProcess_FlipUVs);
      if (!scene) {
        fprintf(stderr, "Could not load resource [%s]: %s",
          resource_path.c_str(), importer.GetErrorString());
        return Ogre::MeshPtr();
      }

      return meshFromAssimpScene(resource_path, scene);
    }
  }

  return Ogre::MeshPtr();
}

}  // namespace rviz
