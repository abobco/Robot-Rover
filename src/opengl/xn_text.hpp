#pragma once
extern "C" {
#include <ft2build.h>
#include FT_FREETYPE_H
}

#include <glm/glm.hpp>
// #include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <map>

namespace xn {

struct FontFace {
    FT_Face face;
    unsigned size;
    FontFace(const char *filename, FT_Library ft, unsigned size = 48) {
        this->size = size;
        if (FT_New_Face(ft, filename, 0, &face)) {
            std::cout << "ERROR: Failed to load font\n";
            return;
        }
        FT_Set_Pixel_Sizes(face, 0, size);

        char c = 'X';
        if (FT_Load_Char(face, c, FT_LOAD_RENDER)) {
            std::cout << "ERROR: Failed to load glyph\n";
            return;
        }

        std::cout << filename << " load success\n";
    }
};

struct Character {
    unsigned texture_id;
    glm::ivec2 size;
    glm::ivec2 bearing;
    long advance;
};

struct Font {
    unsigned size = 0;
    std::map<char, Character> characters;
    unsigned VAO;
    unsigned VBO;

    Font() {}

    Font(FontFace &ff) {
        FT_Face &f = ff.face;
        this->size = ff.size;
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // disable byte-alignment restriction

        for (unsigned char c = 0; c < 128; c++) {
            // std::cout << c << '\n';
            // load character glyph
            if (FT_Load_Char(f, c, FT_LOAD_RENDER)) {
                std::cout << "ERROR::FREETYTPE: Failed to load Glyph" << std::endl;
                continue;
            }
            // generate texture
            unsigned int texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, f->glyph->bitmap.width, f->glyph->bitmap.rows, 0,
                         GL_RED, GL_UNSIGNED_BYTE, f->glyph->bitmap.buffer);
            // set texture options
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // now store character for later use
            Character character = {
                texture, glm::ivec2(f->glyph->bitmap.width, f->glyph->bitmap.rows),
                glm::ivec2(f->glyph->bitmap_left, f->glyph->bitmap_top), f->glyph->advance.x};
            characters.insert(std::pair<char, Character>(c, character));
        }
        // glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

        FT_Done_Face(f);
        // FT_Done_FreeType(ft);
        glGenVertexArrays(1, &VAO);
        glGenBuffers(1, &VBO);
        glBindVertexArray(VAO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        glBindVertexArray(0);
    }
};
} // namespace xn