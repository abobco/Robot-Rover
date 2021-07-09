#pragma once

#include "robo_imgui_shared.hpp"

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/rotate_vector.hpp>

#include <graphics/vulkan/xn_vk_imgui.hpp>
#include <graphics/vulkan/xn_vk_vertex_arrays.hpp>
#include <graphics/vulkan/xn_vulkan_renderer.hpp>

namespace xn {
namespace vulkan {

static void ImGuiImage(const Image &image,
                       glm::uvec2 dimensions = glm::uvec2(240)) {
  // create descriptor sets
  ImTextureID tid = (ImTextureID)ImGui_ImplVulkan_AddTexture(
      image.sampler, image.view, image.layout);

  // draw image
  ImGui::Image(tid, ImVec2((float)dimensions.x, (float)dimensions.y));
}

static void updateDrawNodeMenu(DrawNode *node) {
  static const unsigned gui_buf_len = 124;
  char buf1[gui_buf_len], buf3[gui_buf_len];
  snprintf(buf1, sizeof(buf1), "ID: %d, children: %d", node->id,
           node->countChildrenRecursive());
  snprintf(buf3, sizeof(buf3), "##%d", node->id);
  ImGui::Checkbox(buf3, &node->visible);
  ImGui::SameLine();
  if (ImGui::TreeNode(buf1)) {
    for (unsigned i = 0; i < node->meshes.size(); i++) {
      char buf2[gui_buf_len];
      snprintf(buf2, sizeof(buf2), "Mesh(%d): [vertices] %zu * %zu bytes", i,
               node->meshes[i]->vertices.size(),
               sizeof(node->meshes[i]->vertices[0]));
      ImGui::Text(buf2);
    }
    for (DrawNode *c : node->children) {
      updateDrawNodeMenu(c);
    }
    ImGui::TreePop();
  }
}

static void drawRenderTreeGui(void *argv) {
  ImVec2 win_size = ImGui::GetMainViewport()->Size;
  ImGui::Text("Window Size:( %d, %d )", (int)win_size.x, (int)win_size.y);
  Scene *scene_ptr = (Scene *)argv;
  if (ImGui::TreeNode("Render Tree")) {
    DrawNode *n = &scene_ptr->root;
    updateDrawNodeMenu(n);
    ImGui::TreePop();
  }
}

static ImTextureID ImGuiAddImage(const Image &image) {
  return (ImTextureID)ImGui_ImplVulkan_AddTexture(image.sampler, image.view,
                                                  image.layout);
}
} // namespace vulkan
} // namespace xn