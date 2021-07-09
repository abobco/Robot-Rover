#define XN_EGL

#include "xn_gl.hpp"

using namespace xn;
using namespace gl;

int Init(CUBE_STATE_T *p_state)
{
    UserData *user_data = (UserData *) p_state->user_data;
    GLbyte vShaderStr[] =
	"attribute vec4 vPosition;\n"
	"void main()\n"
	"{\n"
	"gl_Position = vPosition; \n"
	"}\n";
    GLbyte fShaderStr[] =
	"precision mediump float;\n"
	"void main()\n"
	"{\n"
	" gl_FragColor = vec4(1.0, 0.0, 0.0, 1.0); \n"
	"}\n";
    GLuint vertexShader;
    GLuint fragmentShader;
    GLuint programObject;
    GLint linked;

    // Load the vertex/fragment shaders
    vertexShader = LoadShader(GL_VERTEX_SHADER,(const char *) vShaderStr);
    fragmentShader = LoadShader(GL_FRAGMENT_SHADER,(const char *) fShaderStr);
    // Create the program object
    programObject = glCreateProgram();
    if(programObject == 0)
	return 0;
    glAttachShader(programObject, vertexShader);
    glAttachShader(programObject, fragmentShader);
    // Bind vPosition to attribute 0
    glBindAttribLocation(programObject, 0, "vPosition");
    // Link the program
    glLinkProgram(programObject);
    // Check the link status
    glGetProgramiv(programObject, GL_LINK_STATUS, &linked);
    if(!linked)
	{
	    GLint infoLen = 0;
	    glGetProgramiv(programObject, GL_INFO_LOG_LENGTH, &infoLen);
	    if(infoLen > 1)
		{
		    char* infoLog = (char*) malloc(sizeof(char) * infoLen);
		    glGetProgramInfoLog(programObject, infoLen, NULL, infoLog);
		    fprintf(stderr, "Error linking program:\n%s\n", infoLog);
		    free(infoLog);
		}
	    glDeleteProgram(programObject);
	    return FALSE;
	}
    // Store the program object
    user_data->programObject = programObject;
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    return TRUE;
}

///
// Draw a triangle using the shader pair created in Init()
//
void Draw(CUBE_STATE_T *p_state)
{
    static float x = 0;
    float iter = 0.01;
    if ( x+iter > 1 )
        iter*=-1;
    x+=iter;
    printf("%f\n", x);
    UserData *user_data = (UserData*) p_state->user_data;
    GLfloat vVertices[] = {x, 0.5f, 0.0f,
			   -0.5f, -0.5f, 0.0f,
			   0.5f, -0.5f, 0.0f};
    // Set the viewport
    glViewport(0, 0, p_state->width, p_state->height);
    // Clear the color buffer
    glClear(GL_COLOR_BUFFER_BIT);
    // Use the program object
    glUseProgram(user_data->programObject);
    // Load the vertex data
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, vVertices);
    glEnableVertexAttribArray(0);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    eglSwapBuffers(p_state->display, p_state->surface);
}


CUBE_STATE_T state, *p_state = &state;

void  esMainLoop (CUBE_STATE_T *esContext )
{
    struct timeval t1, t2;
    struct timezone tz;
    float deltatime;
    float totaltime = 0.0f;
    unsigned int frames = 0;

    gettimeofday ( &t1 , &tz );

    while(1)
    {
        gettimeofday(&t2, &tz);
        deltatime = (float)(t2.tv_sec - t1.tv_sec + (t2.tv_usec - t1.tv_usec) * 1e-6);
        t1 = t2;

        if (esContext->draw_func != NULL)
            esContext->draw_func(esContext);
        
        // glFlush();
        // time_sleep(1);
        // eglSwapBuffers(esContext->display, esContext->surface); 

        totaltime += deltatime;
        frames++;
        if (totaltime >  2.0f)
        {
            printf("%4d frames rendered in %1.4f seconds -> FPS=%3.4f\n", frames, totaltime, frames/totaltime);
            totaltime -= 2.0f;
            frames = 0;
        }
    }
}

int main(int argc, char *argv[])
{
    UserData user_data;

    bcm_host_init();
    InitContext(p_state);
    EGLinit(p_state, 320, 240);

    p_state->user_data = &user_data;

    if(!Init(p_state))
	return 0;
    RegisterDrawFunc(p_state, Draw);

    eglSwapBuffers(p_state->display, p_state->surface);
    esMainLoop(p_state);
}