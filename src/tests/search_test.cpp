//  g++ ./glad/src/glad.c search_test.cpp -o astar -I./glad/include -lglfw3 -lGL -lX11 -lpthread -lXrandr -lXi -ldl -lXxf86vm -g -Wall
#define PIO_VIRTUAL

#include "xn_gl.hpp"
#include "../xn_vec.hpp"
#include "../xn_search.hpp"
using namespace xn;

#define SCR_WIDTH 800
#define SCR_HEIGHT 800
gl::Camera cam;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

int main (int argc, char **argv) {
    std_vec2d<bool> graph({
        {0,0,0,0,0,0,0,0},
        {0,1,1,1,1,1,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,1,1,1,1,1,0},
        {0,1,1,1,1,1,1,0},
        {0,1,1,1,1,1,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,0,0,0,0,1,0},
        {0,1,1,1,1,1,1,0},
        {0,1,1,1,1,1,1,0},
        {0,0,0,0,0,0,0,0},
    });

    GridNode start(1,1);
    GridNode goal(graph.size()-2, graph.front().size()-2);

    DUMP(goal.x)
    DUMP(goal.y)

    auto path = A_star(graph, start, goal);

    for ( auto n : path )
        printf("(%d, %d)\n", n.x, n.y);

    // init opengl
    GLFWwindow *window = xn::gl::CreateWindow(SCR_WIDTH, SCR_HEIGHT, "boxin");
    Shader cam_shader("shaders/camera.vs", "shaders/camera.fs");

    gl::VertexArrayInfo grid_squares, grid;
    gl::gen_arrays(gl::cube_verts, sizeof(gl::cube_verts), 5, grid_squares);
    gl::gen_arrays(gl::grid_verts, sizeof(gl::grid_verts), 3, grid);
    cam_shader.use();

    while(!glfwWindowShouldClose(window)) {
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glm::mat4 projection;
        glm::mat4 view;

        float orthzoom = 25.0f;
        projection = glm::ortho(-orthzoom, orthzoom, -orthzoom, orthzoom, cam.rad, 10000.0f);
        view = glm::lookAt(glm::vec3(16,6,16), glm::vec3(16,0,16), glm::vec3(0,0,-1));

        cam_shader.setMat4("projection", projection);
        cam_shader.setMat4("view", view);

        glBindVertexArray(grid_squares.VAO);        
        cam_shader.setVec4("color", glm::vec4(0.2f, 0.2f, 0.2f, 1.0f));
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glm::vec3 def_scale(1);
        glm::vec3 origin(0);
        glm::vec3 pos;
        
        for ( unsigned i =0; i < graph.size(); i++ ) 
            for ( unsigned j =0; j < graph[i].size(); j++ ) 
                if ( !graph[i][j]) {
                    pos = {j,0,i};
                    gl::draw_arrays(pos, def_scale, cam_shader, 36); 
                }
        
        cam_shader.setVec4("color", glm::vec4(0.2f, 0.8f, 0.2f, 1.0f));
        for ( auto cell : path ) {
            pos = {cell.y, 0, cell.x};
            gl::draw_arrays(pos, def_scale, cam_shader, 36);
        }

        pos = {start.y, 0, start.x};
        cam_shader.setVec4("color", glm::vec4(0.2f, 0.2f, 0.8f, 1.0f));
        gl::draw_arrays(pos, def_scale, cam_shader, 36);

        pos = {goal.y, 0, goal.x};
        cam_shader.setVec4("color", glm::vec4(0.8f, 0.2f, 0.2f, 1.0f));
        gl::draw_arrays(pos, def_scale, cam_shader, 36);

        gl::draw_grid(grid.VAO, cam_shader,1, 40, 40);
        

        glfwSwapBuffers(window);
        glfwPollEvents();
        
        double remaining_time = 1.0/60 - (glfwGetTime() - currentFrame);
        if ( remaining_time > 0 ) time_sleep(remaining_time);
    }

    glfwTerminate();
}