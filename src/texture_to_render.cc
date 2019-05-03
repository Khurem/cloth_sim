#include <GL/glew.h>
#include <debuggl.h>
#include <iostream>
#include "texture_to_render.h"

TextureToRender::TextureToRender()
{
}

TextureToRender::~TextureToRender()
{
	if (fb_ < 0)
		return ;
	unbind();
	glDeleteFramebuffers(1, &fb_);
	glDeleteTextures(1, &tex_);
	glDeleteRenderbuffers(1, &dep_);
}

void TextureToRender::create(int width, int height)
{
	w_ = width;
	h_ = height;
	// FIXME: Create the framebuffer object backed by a texture
    GLuint FramebufferName = 0;
glGenFramebuffers(1, &fb_);
glBindFramebuffer(GL_FRAMEBUFFER, fb_);
// The texture we're going to render to
GLuint renderedTexture;
glGenTextures(1, &tex_);

// "Bind" the newly created texture : all future texture functions will modify this texture
glBindTexture(GL_TEXTURE_2D, tex_);

// Give an empty image to OpenGL ( the last "0" )
glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, w_, h_, 0,GL_RGB, GL_UNSIGNED_BYTE, 0);

// Poor filtering. Needed !
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

glGenRenderbuffers(1, &dep_);
glBindRenderbuffer(GL_RENDERBUFFER, dep_);
glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, w_, h_);
glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, dep_);
// Set "renderedTexture" as our colour attachement #0
glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, tex_, 0);

// Set the list of draw buffers.
GLenum DrawBuffers[1] = {GL_COLOR_ATTACHMENT0};
glDrawBuffers(1, DrawBuffers); // "1" is the size of DrawBuffers
	
	unbind();
}

void TextureToRender::bind()
{
	// FIXME: Unbind the framebuffer object to GL_FRAMEBUFFER
    // printf("annie can you hear me?\n");
    glBindFramebuffer(GL_FRAMEBUFFER, fb_);
	glViewport(0, 0, w_, h_);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_MULTISAMPLE);
	glEnable(GL_BLEND);
	glEnable(GL_CULL_FACE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDepthFunc(GL_LESS);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glCullFace(GL_BACK);
}

void TextureToRender::unbind()
{
	// FIXME: Unbind current framebuffer object from the render target
    glBindFramebuffer(GL_FRAMEBUFFER, 0); 
}