/*
 * vitaGL stand-in for the glad symbols gfx_opengl.cpp expects. vitaGL links
 * its GL functions directly (no loader), so this just maps the glad_gl*
 * names this file calls onto the real ones, and stubs the GLAD_GL_*
 * capability flags to fixed values instead of runtime extension queries.
 * Unverified against real hardware — see gfx_opengl.cpp's __vita__ branches.
 */
#ifndef PORT_VITA_GL_COMPAT_H
#define PORT_VITA_GL_COMPAT_H

#include <vitaGL.h>

/* vitaGL only has the older EXT-suffixed names for these. */
#define GL_MIRROR_CLAMP_TO_EDGE GL_MIRROR_CLAMP_EXT
#define GL_TEXTURE_MAX_ANISOTROPY GL_TEXTURE_MAX_ANISOTROPY_EXT
#define GL_MAX_TEXTURE_MAX_ANISOTROPY GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT

static const struct { int major, minor; } GLVersion = { 2, 1 };

#define GLAD_GL_ARB_depth_clamp 0
#define GLAD_GL_EXT_depth_clamp 0
#define GLAD_GL_NV_depth_clamp 0
#define GLAD_GL_ARB_texture_mirror_clamp_to_edge 0
#define GLAD_GL_EXT_texture_mirror_clamp_to_edge 0
#define GLAD_GL_ARB_framebuffer_object 1 /* vitaGL provides these as real functions */
#define GLAD_GL_EXT_framebuffer_object 0
#define GLAD_GL_EXT_framebuffer_blit 0
#define GLAD_GL_EXT_framebuffer_multisample 0
#define GLAD_GL_EXT_gpu_shader4 0
#define GLAD_GL_KHR_debug 0

#define glad_glGenFramebuffers glGenFramebuffers
#define glad_glGenRenderbuffers glGenRenderbuffers
#define glad_glDeleteFramebuffers glDeleteFramebuffers
#define glad_glDeleteRenderbuffers glDeleteRenderbuffers
#define glad_glBindFramebuffer glBindFramebuffer
#define glad_glBindRenderbuffer glBindRenderbuffer
#define glad_glFramebufferRenderbuffer glFramebufferRenderbuffer
#define glad_glFramebufferTexture2D glFramebufferTexture2D
#define glad_glRenderbufferStorage glRenderbufferStorage
#define glad_glBlitFramebuffer glBlitFramebuffer
#define glad_glRenderbufferStorageMultisample glRenderbufferStorageMultisample

/* vitaGL has no MSAA renderbuffers at all — fall back to non-MSAA storage
 * rather than leave the symbol undefined. */
static inline void glRenderbufferStorageMultisample(GLenum target, GLsizei samples,
                                                     GLenum internalformat,
                                                     GLsizei width, GLsizei height) {
    (void)samples;
    glRenderbufferStorage(target, internalformat, width, height);
}

#endif /* PORT_VITA_GL_COMPAT_H */
