#pragma once
#include "xn_gl.hpp"
#include "xn_text.hpp"
#include <string>

namespace xn {
struct Button {
    glm::vec2 position;
    std::string text;
    void (*callback)(void);
    gl::VertexArrayInfo vertex_arrays;
    Font font;

    Button(glm::vec2 position, std::string text, Font font, void (*callback)(void)) {
        this->position = position;
        this->text = text;
        this->callback = callback;
        this->font = font;
        gl::gen_arrays(gl::quad_verts, sizeof(gl::quad_verts), vertex_arrays);
    }

    void draw(Shader &geom_shader, Shader &text_shader, const glm::mat4 &text_projection) {
        geom_shader.use();
        geom_shader.setVec2("position", position + glm::vec2(0.08, 0));
        geom_shader.setVec2("scale", {0.2, 0.16}); // get font width from freetype
        glBindVertexArray(vertex_arrays.VAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);

        gl::draw_text(font, text_shader, text, (position.x + 1) * 400, (position.y + 1) * 400,
                      text_projection);
    }
};
} // namespace xn