#include "../opengl/xn_renderer.hpp"
#include "json.hpp"
#include "load_settings.hpp"
#include <fstream>
#include <iostream>
#include <streambuf>
#include <unordered_map>

// for convenience
using json = nlohmann::json;

namespace xn {

void load_drawnode_json(json j, DrawNode *node = &DrawNode::scene) {
  for (auto &e : j.items()) {
    std::cout << e << "\n\n";
    json je = e.value();
    if (!je["Shader"].is_null()) {
      std::string s = je["Shader"];
      if (DrawNode::shader_map.find(s) == DrawNode::shader_map.end()) {
        std::cout << s << std::endl;
        // const char *vs = (s + ".vs").c_str(), *fs = (s + ".fs").c_str();
        std::string vs = s + ".vs", fs = s + ".fs";
        DrawNode::shader_map[s] = Shader(vs.c_str(), fs.c_str());
      }
      DrawNode *d = node->addChild(new DrawNode(&DrawNode::shader_map[s]));
      // for (auto &u : je["Uniforms"].items()) {
      //   std::cout << u.key() << ": ";
      //   if (u.value().is_array()) {
      //     DUMP(u.value().size());
      //     switch (u.value().size()) {
      //     case 2: {
      //       std::vector<float> v(u.value().get<std::vector<float>>);
      //       glm::vec2 vbuf = glm::make_vec2(v);
      //       // std::copy((float *)&vbuf[0], (float *)&u.value()[0],
      //       //           u.value().size() * sizeof(float));
      //       d->addUniform(u.key(), vbuf);
      //     } break;
      //     case 3: {
      //       // std::vector<float> v(u.value());
      //       // glm::vec3 vbuf = glm::make_vec3(v);
      //       // std::copy((float *)&vbuf[0], (float *)&u.value()[0],
      //       //           u.value().size() * sizeof(float));
      //       // d->addUniform(u.key(), vbuf);
      //     } break;
      //     case 4: {
      //       // std::vector<float> v(u.value());
      //       // glm::vec4 vbuf = glm::make_vec4(v);
      //       // std::copy((float *)&vbuf[0], (float *)&u.value()[0],
      //       //           u.value().size() * sizeof(float));
      //       // d->addUniform(u.key(), vbuf);
      //     } break;
      //     default:
      //       break;
      //     }
      //   }
      // }

      DrawNode::node_map[e.key()] = d;
    } else {
      DrawNode::node_map[e.key()] = node->addChild(new DrawNode());
    }
    if (!je["Children"].is_null()) {
      for (auto &c : je["Children"]) {
        load_drawnode_json(c, DrawNode::node_map[e.key()]);
      }
    }
  }
}

void load_scene_json(std::string filepath) {
  json j = load_json_file(filepath);
  if (j["RenderTree"]["Root"].is_null()) {
    std::cout << "invalid config file\n";
    return;
  }

  load_drawnode_json(j["RenderTree"]["Root"]);
}

} // namespace xn