#include "Graphics/Texture.h"
#include <stb_image.h>

namespace Runtime { namespace Graphics {

Texture::~Texture() {
    if (m_handle) {
        glDeleteTextures(1, &m_handle);
        m_handle = 0;
    }
}

bool Texture::LoadFromMemory(const unsigned char* bytes, size_t length, bool flipVertically, bool useSrgb) {
    if (!bytes || length == 0) {
        return false;
    }

    stbi_set_flip_vertically_on_load(flipVertically ? 1 : 0);
    unsigned char* data = stbi_load_from_memory(bytes, static_cast<int>(length), &m_width, &m_height, &m_channels, 0);
    if (!data) {
        return false;
    }

    const GLenum format = (m_channels == 1) ? GL_RED : (m_channels == 4 ? GL_RGBA : GL_RGB);
    GLenum internalFormat = format;
    if (useSrgb) {
        if (m_channels == 3) {
            internalFormat = GL_SRGB;
        } else if (m_channels == 4) {
            internalFormat = GL_SRGB_ALPHA;
        }
    }

    if (m_handle) {
        glDeleteTextures(1, &m_handle);
        m_handle = 0;
    }

    glGenTextures(1, &m_handle);
    glBindTexture(GL_TEXTURE_2D, m_handle);
    glTexImage2D(GL_TEXTURE_2D, 0, static_cast<GLint>(internalFormat), m_width, m_height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_2D, 0);
    stbi_image_free(data);
    return true;
}

void Texture::Bind(int unit) const {
    if (!m_handle) return;
    glActiveTexture(GL_TEXTURE0 + unit);
    glBindTexture(GL_TEXTURE_2D, m_handle);
}

void Texture::Unbind() const {
    glBindTexture(GL_TEXTURE_2D, 0);
}

} }
