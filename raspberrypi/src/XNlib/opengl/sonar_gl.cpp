//  g++ ./glad/src/glad.c sonar_gl.cpp -I./glad/include -lglfw3 -lGL -lX11 -lpthread -lXrandr -lXi -ldl -lXxf86vm -o sonar -g
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include "xn_gl.hpp"
#include "../xn_vec.hpp"
#include <math.h>
#include <pthread.h>

/*
 
  sonar test:
  
  round pole:
    ang=11.5deg
    d=19 cm 
*/

using namespace xn;

#define SCR_WIDTH 800
#define SCR_HEIGHT 800
#define PORT 4000
// #define DUMP(a) { std::cout << #a " = " << (a) << std::endl; } 

int sockfd, newsockfd, portno;

const float us_to_cm = (34300)/(1e6f);
const float deg_to_rad = M_PI / 180.0f;

const float test_tri_hyp = 19;
const float test_tri_ang = 11.5*deg_to_rad;
const float test_tri_x = cos(test_tri_ang)*test_tri_hyp;
const float test_tri_y = sin(test_tri_ang)*test_tri_hyp;  

// camera
float camera_ang = M_PI/2; 
float camera_rad = 3;
float camera_height = 6;
glm::vec3 cameraPos   = glm::vec3(cosf(camera_ang)*camera_rad, camera_height, sinf(camera_ang)*camera_rad);
glm::vec3 cameraFront = glm::vec3(1.0f, 0.0f, 0.0f);
glm::vec3 cameraUp    = glm::vec3(0.0f, 1.0f, 0.0f);

bool firstMouse = true, run=1;
float yaw   = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch =  0.0f;

// timing
float deltaTime = 0.0f;	// time between current frame and last frame
float lastFrame = 0.0f;

pthread_mutex_t mut;
const int samples_max = 4096;
int samples_count =0;
glm::vec3 samples[samples_max];

void time_sleep(double seconds)
{
   struct timespec ts, rem;

   if (seconds > 0.0)
   {
      ts.tv_sec = seconds;
      ts.tv_nsec = (seconds-(double)ts.tv_sec) * 1E9;

      while (clock_nanosleep(CLOCK_REALTIME, 0, &ts, &rem))
      {
         /* copy remaining time to ts */
         ts.tv_sec  = rem.tv_sec;
         ts.tv_nsec = rem.tv_nsec;
      }
   }
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void read_uint32_t(int sock, uint32_t *intptr ) {
      uint32_t tmp; 
      uint32_t left = sizeof(*intptr);
      char *buf = (char *) &tmp;
      int rc;
      do {
        rc =  recv(sock, buf, left, 0);
        buf += rc;
        left -= rc;
      } while (left > 0);
      *intptr = ntohl(tmp);
}

void *sonar_io_thread(void *argv) {
  while(run) {
    uint32_t val, step, servo_ang_i;
    read_uint32_t(newsockfd, &val );
    read_uint32_t(newsockfd, &step );
    read_uint32_t(newsockfd, &servo_ang_i );
    // DUMP(val);
    // DUMP(step);
    
    // double val_norm = val*1.0/18500;
    float servo_ang_f = servo_ang_i/1000.0f;
    double ang = (step*(1.0/100)*M_PI);
    double val_norm = val*1.0/4625;
    float val_cm = us_to_cm*val*0.5;
    val_norm = val_cm/50;
      if ( val_norm < 0.5 ) {
      printf("OBJECT CLOSE ");
    }
    // DUMP(val_norm);
    DUMP(servo_ang_f);

    pthread_mutex_lock(&mut);
    vec3 v = {val_norm, 0, 0};
    v = vec3::rotate_axis(v, {0,1,0}, ang);
    v = vec3::rotate_axis(v, {1,0,0}, servo_ang_f);
    samples[samples_count] = glm::vec3(v.x, v.y, v.z);
    // samples[samples_count] = glm::vec3(cos(-ang)*val_norm, 0, sin(-ang)*val_norm );
    samples_count++;
    pthread_mutex_unlock(&mut);
  }
  return NULL;
}

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
      glfwSetWindowShouldClose(window, true);
      run = 0;
    }

    float cameraSpeed = 2.5 * deltaTime;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
      camera_rad -= cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
      camera_rad += cameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
      camera_ang += cameraSpeed;
    }
        // cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
      camera_ang -= cameraSpeed;
    }    

    // static bool edown = false;
    // if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS && !edown ) {
    //   mode = (mode+1) % max_mode;
    //   edown = true;
    // } else if ( glfwGetKey(window, GLFW_KEY_E) == GLFW_RELEASE ) {
    //   edown = false;
    // }
    cameraPos = glm::vec3(cosf(camera_ang)*camera_rad, camera_height, sinf(camera_ang)*camera_rad); 
}

int main(int argc, char *argv[])
{
    // Shader shader("shaders/2d.vs", "shaders/2d.fs");
    // shader.use();

    socklen_t clilen;
    char buffer[256];
    struct sockaddr_in serv_addr, cli_addr;
    int n;
    if (argc < 2) {
        fprintf(stderr,"ERROR, no port provided\n");
        exit(1);
    }
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
      error("ERROR opening socket");
    bzero((char *) &serv_addr, sizeof(serv_addr));
    portno = atoi(argv[1]);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(portno);
    if (bind(sockfd, (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)) < 0) 
            error("ERROR on binding");
    listen(sockfd,5);
    clilen = sizeof(cli_addr);
    newsockfd = accept(sockfd, 
                (struct sockaddr *) &cli_addr, 
                &clilen);
    if (newsockfd < 0) 
        error("ERROR on accept");
      
    if (pthread_mutex_init(&mut, NULL) != 0) { 
        printf("\n mutex init has failed\n"); 
        return 1; 
    } 
    pthread_t tid;
    pthread_create(&tid, NULL, &sonar_io_thread, NULL);

    // float vert[] = {
    //      0.5f,  0.5f, 0.0f,  // top right
    //     -0.5f, -0.5f, 0.0f,  // bottom left
    //     -0.5f,  0.5f, 0.0f   // top left 
    // };
    // unsigned int indices[] = {  // note that we start from 0!
    //     0, 1, 2,  // first Triangle
    // };
    // unsigned int VBO, VAO, EBO;
    // glGenVertexArrays(1, &VAO);
    // glGenBuffers(1, &VBO);
    // glGenBuffers(1, &EBO);
    // // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    // glBindVertexArray(VAO);

    // glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(vert), vert, GL_STATIC_DRAW);

    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    // glEnableVertexAttribArray(0);

    // // note that this is allowed, the call to glVertexAttribPointer registered VBO as the vertex attribute's bound vertex buffer object so afterwards we can safely unbind
    // glBindBuffer(GL_ARRAY_BUFFER, 0); 

    // remember: do NOT unbind the EBO while a VAO is active as the bound element buffer object IS stored in the VAO; keep the EBO bound.
    //glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

    // You can unbind the VAO afterwards so other VAO calls won't accidentally modify this VAO, but this rarely happens. Modifying other
    // VAOs requires a call to glBindVertexArray anyways so we generally don't unbind VAOs (nor VBOs) when it's not directly necessary.
    // glBindVertexArray(0); 

    // uncomment this call to draw in wireframe polygons.
    // glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // vert[0]=0;
    // vert[1]=0,
    // vert[2]=0;
    // vert[3]=0 + 0.005;
    // vert[4]=0,
    // vert[5]=0;
    // vert[6]=0;
    // vert[7]=0 + 0.005,
    // vert[8]=0;

    // glBindBuffer(GL_ARRAY_BUFFER, VBO);
    // glBufferData(GL_ARRAY_BUFFER, sizeof(vert), vert, GL_STATIC_DRAW);

    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);


    // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    // glEnableVertexAttribArray(0);

    // glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    // glClear(GL_COLOR_BUFFER_BIT);
    // shader.use();

    // glBindVertexArray(VAO);
    // glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, 0);

    // glfwSwapBuffers(window);
    // glfwPollEvents();

    
    // init opengl
    GLFWwindow *window = xn::gl::CreateWindow(SCR_WIDTH, SCR_HEIGHT, "boxin");

    glEnable(GL_DEPTH_TEST);
    Shader cam_shader("shaders/camera.vs", "shaders/camera.fs");
    // set_bone_positions(phys_arm);
    unsigned int VBO, VAO, VBO_grid, VAO_grid;
    gl::gen_arrays(&VAO, &VBO, gl::cube_verts, sizeof(gl::cube_verts), 5);
    gl::gen_arrays(&VAO_grid, &VBO_grid, gl::grid_verts, sizeof(gl::grid_verts), 3);
    cam_shader.use();
    
    while(!glfwWindowShouldClose(window) ) {
        // uint32_t val, step;
        // read_uint32_t(newsockfd, &val );
        // read_uint32_t(newsockfd, &step );
        // DUMP(val);
        // DUMP(step);

        // // double val_norm = val*1.0/18500;
        
        // double ang = (step*(1.0/100)*M_PI);
        // double val_norm = val*1.0/4625;
        // float val_cm = us_to_cm*val*0.5;
        // val_norm = val_cm/100;
        //   if ( val_norm < 0.5 ) {
        //   printf("OBJECT CLOSE ");
        // }
        // DUMP(val_norm);

        // samples[samples_count] = glm::vec3(cos(ang)*val_norm, 0, sin(ang)*val_norm );
        // samples_count++;

        // // double ang = 0;
        // DUMP(ang);
        // vert[0]=cosf(ang)*val_norm;
        // vert[1]=sinf(ang)*val_norm,
        // vert[2]=0;
        // vert[3]=cosf(ang)*val_norm + 0.005;
        // vert[4]=sinf(ang)*val_norm,
        // vert[5]=0;
        // vert[6]=cosf(ang)*val_norm;
        // vert[7]=sinf(ang)*val_norm + 0.005,
        // vert[8]=0;

        // glBindBuffer(GL_ARRAY_BUFFER, VBO);
        // glBufferData(GL_ARRAY_BUFFER, sizeof(vert), vert, GL_STATIC_DRAW);

        // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        // glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);


        // glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        // glEnableVertexAttribArray(0);

        // // glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        // // glClear(GL_COLOR_BUFFER_BIT);
        // shader.use();

        // glBindVertexArray(VAO);
        // glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT, 0);

        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        processInput(window);
        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glm::mat4 projection = glm::perspective(glm::radians(xn::gl::fov), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        cam_shader.setMat4("projection", projection);
        glm::mat4 view = glm::lookAt(cameraPos, glm::vec3(0,1,0), cameraUp);
        cam_shader.setMat4("view", view);
        glBindVertexArray(VAO);

                
        cam_shader.setVec4("color", glm::vec4(0.2f, 0.7f, 0.2f, 1.0f));
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glm::vec3 def_scale{0.05,0.05,0.05};
        glm::vec3 origin{0,0,0};
        gl::draw_arrays(origin, def_scale, cam_shader, 36);

        cam_shader.setVec4("color", glm::vec4(1.0f, 0.5f, 0.2f, 1.0f));
        pthread_mutex_lock(&mut);
        for ( int i =0; i < samples_count; i++ ) {
          gl::draw_arrays(samples[i], def_scale, cam_shader, 36);
        }
        pthread_mutex_unlock(&mut);

        gl::draw_grid(VAO_grid, cam_shader);

        glfwSwapBuffers(window);
        glfwPollEvents();

        time_sleep(1.0/60);
    }

    
    close(newsockfd);
    close(sockfd);
    glfwTerminate();
    pthread_join(tid, NULL);
     return 0; 
}
