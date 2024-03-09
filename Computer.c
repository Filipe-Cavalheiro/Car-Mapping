#include <unistd.h>
#include <math.h>
#include "shader_lib.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "cglm/include/cglm/cglm.h"
#include "port.h"
#include "../Data-structures-in-C/linkedList.h"
#include "cube.h"

/*
#include <ft2build.h>
#include FT_FREETYPE_H
*/

#define SCR_WIDTH 800
#define SCR_HEIGHT 600
#define FOV M_PI/4
#define CAMERASPEED 2.5
#define SENSITIVITY 0.1
#define RX_BUFFER_SIZE 64
#define ARROWKEYS '-'

/*
   typedef struct{
   unsigned int TextureID;  // ID handle of the glyph texture
   vec2 Size;       // Size of glyph
   vec2 Bearing;    // Offset from baseline to left/top of glyph
   unsigned int Advance;    // Offset to advance to next glyph
   }Character;
   */

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void processInput(GLFWwindow *window);
float toRad(float deg);
//Character* makeCharacter(unsigned int texture,FT_Face face);
//int addCharacter(LinkedList *linkedList, unsigned int texture, FT_Face face);
//void RenderText(LinkedList *linkedList, GLuint shader, char text[], float x, float y, float scale, vec3 color);
void set_currently_pressed_key(uint8_t key);
void clear_currently_pressed_key(uint8_t key);
uint8_t get_currently_pressed_key(uint8_t key);

// settings
//---------

//set up camera
vec3 cameraPos    = {0.0f, 2.0f,  5.0f};
vec3 cameraFront  = {0.0f, 0.0f, -1.0f};
vec3 cameraUp     = {0.0f, 1.0f,  0.0f};

//set up car
vec3 carPos = {0.0f , 0.5f, 0.0f};
vec3 carSize = {1.0f , 1.0f, 1.0f};
float carAngle = 0;


uint8_t firstMouse = 1;
float yaw   = -90.0f;	// yaw is initialized to -90.0 degrees since a yaw of 0.0 results in a direction vector pointing to the right so we initially rotate a bit to the left.
float pitch =  0.0f;
float lastX =  400.0f;
float lastY =  300.0f;

float deltaTime = 0;
float lastFrame = 0;
uint8_t portConnection = 0;
uint8_t write_to_port = {0};
uint8_t currently_pressed_keys[30] = {0};
unsigned int VAOText, VBOText;

int main(){
    int fd = open (PORTNAME, O_RDWR | O_NOCTTY | O_NONBLOCK |O_ASYNC);
    if (fd < 0){
        fprintf(stderr, "error %d opening %s: %s\n", errno, PORTNAME, strerror (errno));
    }

    portConnection = set_interface_attribs(fd);   // set speed to 9600 bps, 8N1 (no parity)					      
    if(portConnection == 1){
        char buf [128];
        uint8_t bytes_read;

        //soft restart the atmega328p
        char kickstart = '1';
        write(fd, &kickstart, 1);
    }

    LinkedList *cubesLinkedList = makeLinkedList();
    if(cubesLinkedList == NULL){
        fprintf(stderr, "Failed to create cubesLinkedList\n");
        return -1;
    }

    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL){
        fprintf(stderr, "Failed to create GLFW window\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    // tell GLFW to capture mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)){
        fprintf(stderr, "Failed to initialize GLAD\n");
        return -1;
    }

    // FreeType
    // --------
    /*
       FT_Library ft;
       if (FT_Init_FreeType(&ft)){
       fprintf(stderr, "ERROR-FREETYPE: Could not init FreeType Library \n");
       return -1;
       }

       FT_Face face;
       if (FT_New_Face(ft, "/usr/share/fonts/truetype/crosextra/Caladea-Regular.ttf", 0, &face)){
       fprintf(stderr, "ERROR-FREETYPE: Failed to load font \n");  
       return -1;
       }

       FT_Set_Pixel_Sizes(face, 0, 48);  
       if (FT_Load_Char(face, 'X', FT_LOAD_RENDER)){
       fprintf(stderr, "ERROR::FREETYTPE: Failed to load Glyph \n");  
       return -1;
       }

       glPixelStorei(GL_UNPACK_ALIGNMENT, 1); // disable byte-alignment restriction

       LinkedList *characterLinkedList = makeLinkedList();
       if(characterLinkedList == NULL){
       fprintf(stderr, "Failed to create characterLinkedList");
       return -1;
       }
       for (unsigned char c = 0; c < 128; c++){
    // load character glyph 
    if (FT_Load_Char(face, c, FT_LOAD_RENDER)){
    fprintf(stderr, "ERROR::FREETYTPE: Failed to load Glyph \n");
    continue;
    }
    // generate texture
    unsigned int texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexImage2D(
    GL_TEXTURE_2D,
    0,
    GL_RED,
    face->glyph->bitmap.width,
    face->glyph->bitmap.rows,
    0,
    GL_RED,
    GL_UNSIGNED_BYTE,
    face->glyph->bitmap.buffer
    );
    // set texture options
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // now store character for later use
    addCharacter(characterLinkedList, texture, face);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    // destroy FreeType once we're finished
    FT_Done_Face(face);
    FT_Done_FreeType(ft);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4); // disable byte-alignment restriction


    // configure VAO/VBO for texture quads (text)
    // -----------------------------------
    mat4 projection;
    glm_ortho(0.0f, 800.0f, 0.0f, 600.0f, 0, 0, projection);
    glGenVertexArrays(1, &VAOText);
    glGenBuffers(1, &VBOText);
    glBindVertexArray(VAOText);
    glBindBuffer(GL_ARRAY_BUFFER, VBOText);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, NULL, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 4 * sizeof(float), 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);


    // build and compile shader program (text)
    // ------------------------------------
    GLuint text_vertex = glCreateShader(GL_VERTEX_SHADER);
    compile_shader(&text_vertex, GL_VERTEX_SHADER, "shaders/text.vs");

    GLuint text_frag = glCreateShader(GL_VERTEX_SHADER);
    compile_shader(&text_frag, GL_FRAGMENT_SHADER, "shaders/text.fs");

    GLuint textShader = glCreateProgram();
    link_shader(text_vertex, text_frag, textShader); 
    */

        // build and compile shader program (grid)
        // ------------------------------------
        GLuint gridVS = glCreateShader(GL_VERTEX_SHADER);
    compile_shader(&gridVS, GL_VERTEX_SHADER, "shaders/grid.vs");

    GLuint gridFS = glCreateShader(GL_FRAGMENT_SHADER);
    compile_shader(&gridFS, GL_FRAGMENT_SHADER, "shaders/grid.fs");

    GLuint gridShader = glCreateProgram();
    link_shader(gridVS, gridFS, gridShader);

    float grid_vertices[] = {
        // pos   
        1.0f,  1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,   
        -1.0f, -1.0f, 0.0f,   
        -1.0f,  1.0f, 0.0f, 
    };
    unsigned int indices[] = {
        0, 1, 3, 
        1, 2, 3 
    };
    unsigned int GRID_VBO, GRID_VAO, GRID_EBO;
    glGenVertexArrays(1, &GRID_VAO);
    glGenBuffers(1, &GRID_VBO);
    glGenBuffers(1, &GRID_EBO);

    glBindVertexArray(GRID_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, GRID_VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(grid_vertices), grid_vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, GRID_EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    //Clean Up, free all Vertex Array Objects
    glBindVertexArray(0);

    // build and compile shader program (cube)
    // ------------------------------------
    GLuint cubeVS = glCreateShader(GL_VERTEX_SHADER);
    compile_shader(&cubeVS, GL_VERTEX_SHADER, "shaders/cube.vs");

    GLuint cubeFS = glCreateShader(GL_VERTEX_SHADER);
    compile_shader(&cubeFS, GL_FRAGMENT_SHADER, "shaders/cube.fs");

    GLuint cubeShader = glCreateProgram();
    link_shader(cubeVS, cubeFS, cubeShader);

    float vertices[] = {
        -0.5f, -0.5f, -0.5f,
        0.5f, -0.5f, -0.5f,
        0.5f,  0.5f, -0.5f,  
        0.5f,  0.5f, -0.5f,  
        -0.5f,  0.5f, -0.5f,  
        -0.5f, -0.5f, -0.5f,  

        -0.5f, -0.5f,  0.5f,  
        0.5f, -0.5f,  0.5f,  
        0.5f,  0.5f,  0.5f,  
        0.5f,  0.5f,  0.5f,  
        -0.5f,  0.5f,  0.5f,  
        -0.5f, -0.5f,  0.5f,  

        -0.5f,  0.5f,  0.5f,  
        -0.5f,  0.5f, -0.5f,  
        -0.5f, -0.5f, -0.5f,  
        -0.5f, -0.5f, -0.5f,  
        -0.5f, -0.5f,  0.5f,  
        -0.5f,  0.5f,  0.5f,  

        0.5f,  0.5f,  0.5f,  
        0.5f,  0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,  
        0.5f, -0.5f,  0.5f,  
        0.5f,  0.5f,  0.5f,  

        -0.5f, -0.5f, -0.5f,  
        0.5f, -0.5f, -0.5f,  
        0.5f, -0.5f,  0.5f,  
        0.5f, -0.5f,  0.5f,  
        -0.5f, -0.5f,  0.5f,  
        -0.5f, -0.5f, -0.5f,  

        -0.5f,  0.5f, -0.5f,  
        0.5f,  0.5f, -0.5f,  
        0.5f,  0.5f,  0.5f,  
        0.5f,  0.5f,  0.5f,  
        -0.5f,  0.5f,  0.5f,  
        -0.5f,  0.5f, -0.5f,  
    };

    unsigned int CUBE_VBO, CUBE_VAO;
    glGenVertexArrays(1, &CUBE_VAO);
    glGenBuffers(1, &CUBE_VBO);

    glBindVertexArray(CUBE_VAO);

    glBindBuffer(GL_ARRAY_BUFFER, CUBE_VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    //Clean Up, free all Vertex Array Objects
    glBindVertexArray(0);

    // render loop
    // -----------
    mat4 matrix_model;
    mat4 matrix_view;
    mat4 matrix_projection;
    unsigned int modelLoc;
    unsigned int viewLoc;
    unsigned int projectionLoc;
    float angle;
    float currentFrame;
    vec3 cameraTemp;
    node *no;
    double lastTime = glfwGetTime();
    int nbFrames = 0;
    int n_rx;
    uint8_t rx_data[255];
    float distance;

    while (!glfwWindowShouldClose(window)){
        // per-frame time logic
        // --------------------
        currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        //fps counter
        nbFrames++;
        if ( currentFrame - lastTime >= 1.0 ){ // If last prinf() was more than 1 sec ago
                                               // printf and reset timer
            printf("%d fps\n", nbFrames);
            nbFrames = 0;
            lastTime += 1.0;
        }

        // input
        // -----
        processInput(window);

        memset(rx_data, 0, 255);
        n_rx = read(fd, rx_data, 2);
        if(n_rx > 1){
            distance = rx_data[1] << 8 | rx_data[0];
            printf("read %d containing: %f\n",n_rx, distance);
            distance = distance / 100;
            vec3 detected_position = {carPos[0] - distance * sin(carAngle), 0, carPos[3] - distance * cos(carAngle)} ;
            printf("car position X:%f Z:%f Y:%f\n", carPos[0], carPos[1], carPos[2]);
            printf("Created a cube located at X:%f Z:%f Y:%f\n", detected_position[0], detected_position[1], detected_position[2]);
            addCube(cubesLinkedList, detected_position, (vec3){0.1, 0.1, 0.1}); 
        }

        //output
        //------
        if(write_to_port){
            printf("data to port: %d\n", write_to_port);
            write(fd, &write_to_port, 1);
            write_to_port = 0;
        }

        // render
        // ------
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //enable varied opacity
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE);

        //create transformations(grid)
        glUseProgram(gridShader);

        glm_mat4_identity(matrix_view);
        for(int i = 0; i < 3; ++i){cameraTemp[i] = cameraPos[i] + cameraFront[i];}
        glm_lookat(cameraPos, cameraTemp, cameraUp, matrix_view);
        viewLoc = glGetUniformLocation(gridShader, "view");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, *matrix_view);

        glm_mat4_identity(matrix_projection);
        glm_perspective(FOV, (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f, matrix_projection);
        projectionLoc = glGetUniformLocation(gridShader, "projection");
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, *matrix_projection);

        glBindVertexArray(GRID_VAO);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glDisable(GL_BLEND);

        //activate shaders
        glUseProgram(cubeShader);

        //create transformations(cube)
        //car
        glm_mat4_identity(matrix_model);
        glm_translate(matrix_model, carPos);
        glm_scale(matrix_model, carSize);
        mat4 matrix_tmp = {		
        cos(carAngle), 0, -sin(carAngle), 0,
        0, 1,           0, 0,
        sin(carAngle), 0,  cos(carAngle), 0,
        0, 0,           0, 1,
        };
        glm_mat4_mul(matrix_model,matrix_tmp,matrix_model);
        modelLoc = glGetUniformLocation(cubeShader, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, *matrix_model);

        glm_mat4_identity(matrix_view);
        for(int i = 0; i < 3; ++i){
        cameraTemp[i] = cameraPos[i] + cameraFront[i];
        }
        glm_lookat(cameraPos, cameraTemp, cameraUp, matrix_view);
        viewLoc = glGetUniformLocation(cubeShader, "view");
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, *matrix_view);

        glm_mat4_identity(matrix_projection);
        glm_perspective(FOV, (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f, matrix_projection);
        projectionLoc = glGetUniformLocation(cubeShader, "projection");
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, *matrix_projection);

        glBindVertexArray(CUBE_VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        //walls
        no = cubesLinkedList->head;
        while(no != NULL){
        // calculate the model matrix for each object and pass it to shader before drawing
        glm_mat4_identity(matrix_model);
        glm_translate(matrix_model, ((Cube*)no->data)->position);
        glm_scale(matrix_model, ((Cube*)no->data)->size);
        modelLoc = glGetUniformLocation(cubeShader, "model");
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, *matrix_model);
        glBindVertexArray(CUBE_VAO);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        no = no->next;
        }

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    //dealocate memory
    //glDeleteVertexArrays(1, &VAOText);
    //glDeleteBuffers(1, &VBOText);
    //glDeleteProgram(textShader);
    glDeleteVertexArrays(1, &CUBE_VAO);
    glDeleteBuffers(1, &CUBE_VBO);
    glDeleteProgram(cubeShader);
    glDeleteVertexArrays(1, &GRID_VAO);
    glDeleteBuffers(1, &GRID_VBO);
    glDeleteProgram(gridShader);

    // glfw: terminate, clearing all previously allocated GLFW resources.
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow *window){
    vec3 cameraTemp;
    if(get_currently_pressed_key('q') && glfwGetKey(window, GLFW_KEY_Q) == GLFW_RELEASE){
        clear_currently_pressed_key('q');
    }
    if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){
        if(!get_currently_pressed_key('q')){    
            write_to_port = 10;
            glfwSetWindowShouldClose(window, 1);
            set_currently_pressed_key('q');
        }
    }
    float frameCameraSpeed = deltaTime * CAMERASPEED;
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS){
        for(int i = 0; i < 3; ++i)
            cameraPos[i] += cameraFront[i] * frameCameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS){
        for(int i = 0; i < 3; ++i)
            cameraPos[i] -= cameraFront[i] * frameCameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS){ 
        for(int i = 0; i < 3; ++i)
            cameraTemp[i] = 0;
        glm_vec3_crossn(cameraFront, cameraUp, cameraTemp);
        glm_normalize(cameraTemp);
        for(int i = 0; i < 3; ++i)
            cameraPos[i] -= cameraTemp[i] * frameCameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS){
        for(int i = 0; i < 3; ++i)
            cameraTemp[i] = 0;
        glm_vec3_crossn(cameraFront, cameraUp, cameraTemp);
        glm_normalize(cameraTemp);
        for(int i = 0; i < 3; ++i)
            cameraPos[i] += cameraTemp[i] * frameCameraSpeed;
    } 
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS){
        for(int i = 0; i < 3; ++i)
            cameraPos[i] += cameraUp[i] * frameCameraSpeed;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS){
        for(int i = 0; i < 3; ++i)
            cameraPos[i] -= cameraUp[i] * frameCameraSpeed;
    }
    if(get_currently_pressed_key('m') && glfwGetKey(window, GLFW_KEY_M) == GLFW_RELEASE){
        clear_currently_pressed_key('m');
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
    if(glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS){
        if(!get_currently_pressed_key('m')){
            set_currently_pressed_key('m');
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        }
    }
    if(get_currently_pressed_key(ARROWKEYS) && glfwGetKey(window, GLFW_KEY_UP) == GLFW_RELEASE && glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_RELEASE && glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_RELEASE && glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_RELEASE){
        write_to_port = 10;
        clear_currently_pressed_key(ARROWKEYS);
    }
    if(glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS){
        carPos[2] -= cos(carAngle) * frameCameraSpeed;
        carPos[0] -= sin(carAngle) * frameCameraSpeed;
        if(!get_currently_pressed_key(ARROWKEYS)){
            write_to_port = 11;
            set_currently_pressed_key(ARROWKEYS);
        }
    }
    else if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS){
        carPos[2] += cos(carAngle) * frameCameraSpeed;
        carPos[0] += sin(carAngle) * frameCameraSpeed;
        if(!get_currently_pressed_key(ARROWKEYS)){
            write_to_port = 12;
            set_currently_pressed_key(ARROWKEYS);
        }
    }
    else if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS){
        carAngle += frameCameraSpeed;
        if(!get_currently_pressed_key(ARROWKEYS)){
            write_to_port = 13;
            set_currently_pressed_key(ARROWKEYS);
        }
    }
    else if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS){
        carAngle -= frameCameraSpeed;
        if(!get_currently_pressed_key(ARROWKEYS)){
            write_to_port = 14;
            set_currently_pressed_key(ARROWKEYS);
        }
    }
    if(get_currently_pressed_key('u') && glfwGetKey(window, GLFW_KEY_U) == GLFW_RELEASE){
        clear_currently_pressed_key('u');
    }
    if(glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS){
        if(!get_currently_pressed_key('u')){
            write_to_port = 15;
            set_currently_pressed_key('u');
        }
    }
    if(get_currently_pressed_key('i') && glfwGetKey(window, GLFW_KEY_I) == GLFW_RELEASE){
        clear_currently_pressed_key('i');
    }
    if(glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS){
        if(!get_currently_pressed_key('i')){
            write_to_port = 16;
            set_currently_pressed_key('i');
        }
    }
    if(get_currently_pressed_key('j') && glfwGetKey(window, GLFW_KEY_J) == GLFW_RELEASE){
        clear_currently_pressed_key('j');
    }
    if(glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS){
        if(!get_currently_pressed_key('j')){
            set_currently_pressed_key('j');
            write_to_port = 20;
        }
    }
    if(get_currently_pressed_key('h') && glfwGetKey(window, GLFW_KEY_H) == GLFW_RELEASE){
        clear_currently_pressed_key('h');
    }
    if(glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS){
        if(!get_currently_pressed_key('h')){
            set_currently_pressed_key('h');
            write_to_port = 30;
        }
    }
    if(get_currently_pressed_key('g') && glfwGetKey(window, GLFW_KEY_G) == GLFW_RELEASE){
        clear_currently_pressed_key('g');
    }
    if(glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS){
        if(!get_currently_pressed_key('g')){
            set_currently_pressed_key('g');
            write_to_port = 31;
        }
    }
    if(get_currently_pressed_key('f') && glfwGetKey(window, GLFW_KEY_F) == GLFW_RELEASE){
        clear_currently_pressed_key('f');
    }
    if(glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS){
        if(!get_currently_pressed_key('f')){
            write_to_port = 32;
            set_currently_pressed_key('f');
        }
    }
    if(get_currently_pressed_key('o') && glfwGetKey(window, GLFW_KEY_O) == GLFW_RELEASE){
        clear_currently_pressed_key('o');
    }
    if(glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS){
        if(!get_currently_pressed_key('o')){
            write_to_port = 40;
            set_currently_pressed_key('o');
        }
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
void framebuffer_size_callback(GLFWwindow* window, int width, int height){
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn){
    float xpos = (float)xposIn;
    float ypos = (float)yposIn;

    if (firstMouse){
        lastX = xpos;
        lastY = ypos;
        firstMouse = 0;
        return;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top
    lastX = xpos;
    lastY = ypos;

    xoffset *= SENSITIVITY;
    yoffset *= SENSITIVITY;

    yaw += xoffset;
    pitch += yoffset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    vec3 front;
    front[0] = cos(toRad(yaw)) * cos(toRad(pitch));
    front[1] = sin(toRad(pitch));
    front[2] = sin(toRad(yaw)) * cos(toRad(pitch));
    glm_normalize(front);
    for(int i = 0; i < 3; ++i)
        cameraFront[i] = front[i];
    return;
}

float toRad(float deg){
    return deg * (M_PI/180);
}

/*Character* makeCharacter(unsigned int texture,FT_Face face){ 
  Character *character = (Character*)malloc(sizeof(Character));
  character->TextureID = texture;
  glm_vec2_copy((vec2){face->glyph->bitmap.width, face->glyph->bitmap.rows}, character->Size);
  glm_vec2_copy((vec2){face->glyph->bitmap_left, face->glyph->bitmap_top}, character->Bearing);
  character->Advance = face->glyph->advance.x;
  return character;
  }

  int addCharacter(LinkedList *linkedList, unsigned int texture, FT_Face face){
  if(linkedList->head == NULL){
  linkedList->head = makeNo(makeCharacter(texture, face));
  if(linkedList->head == NULL) return 1;
  linkedList->tail = linkedList->head;
  return 0;
  }
  No *no = makeNo(makeCharacter(texture, face));
  if(no == NULL) return 1;
  linkedList->tail->next = no;
  linkedList->tail = no;
  return 0;
  }

  void RenderText(LinkedList *linkedList, GLuint shader, char text[], float x, float y, float scale, vec3 color){
  perror("err");
// activate corresponding render state	
glUseProgram(shader);
glUniform3f(glGetUniformLocation(shader, "textColor"), color[0], color[1], color[2]);
glActiveTexture(GL_TEXTURE0);
glBindVertexArray(VAOText);

// iterate through all characters
for(int i = 0; text[i] != '\0'; ++i){
No *no = linkedList->head;
for(int j = 0; text[i]-j; ++j){
no = no->next;
}
Character *ch = (Character*)(no->data);
float xpos = x + ch->Bearing[0] * scale;
float ypos = y - (ch->Size[1] - ch->Bearing[1]) * scale;

float w = ch->Size[0] * scale;
float h = ch->Size[1] * scale;
// update VBO for each character
float vertices[6][4] = {
{ xpos,     ypos + h,   0.0f, 0.0f },            
{ xpos,     ypos,       0.0f, 1.0f },
{ xpos + w, ypos,       1.0f, 1.0f },
{ xpos,     ypos + h,   0.0f, 0.0f },
{ xpos + w, ypos,       1.0f, 1.0f },
{ xpos + w, ypos + h,   1.0f, 0.0f }           
};
// render glyph texture over quad
glBindTexture(GL_TEXTURE_2D, ch->TextureID);
// update content of VBO memory
glBindBuffer(GL_ARRAY_BUFFER, VBOText);
glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices); 
glBindBuffer(GL_ARRAY_BUFFER, 0);
// render quad
glDrawArrays(GL_TRIANGLES, 0, 6);
// now advance cursors for next glyph (note that advance is number of 1/64 pixels)
x += (ch->Advance >> 6) * scale; // bitshift by 6 to get value in pixels (2^6 = 64)
}
glBindVertexArray(0);
glBindTexture(GL_TEXTURE_2D, 0);
}*/

void set_currently_pressed_key(uint8_t key){
    if(key == ARROWKEYS){ 
        currently_pressed_keys[27] = 1;
        return;
    }
    currently_pressed_keys[key - 'a'] = 1;
}

void clear_currently_pressed_key(uint8_t key){
    if(key == ARROWKEYS){ 
        currently_pressed_keys[27] = 0;
        return;
    }
    currently_pressed_keys[key - 'a'] = 0;
}

uint8_t get_currently_pressed_key(uint8_t key){
    if(key == ARROWKEYS){ 
        return currently_pressed_keys[27];
    }
    return currently_pressed_keys[key - 'a'];
}
