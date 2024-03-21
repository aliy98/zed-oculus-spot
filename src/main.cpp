/**
 * @file main.cpp
 * @brief This node implements a passthrough from the ZED2 stereo camera to Meta Quest 2.
 *
 * @author Ali Yousefi
 * Contact: ali.yousefi@edu.unige.it
 * 
 * @details
 * 
 * 
 * Description:
 * 
 *  This is a modified version of the software available on zed-oculus from Stereolabs repository. Since the IMU and touch input data
 *  is required for this work, the main.cpp is modified in such a way that it reads the angular velocities, 
 *  and touch input data using ts.HeadPose.AngularVelocity, InputState.Thumbstick[ovrHand_Right], and 
 *  InputState.Thumbstick[ovrHand_Left] class attributes, and sends them to the process executed by main.py. 
 *  Additionally, considering the fact that the ZED camera is not connected to the user PC with a USB cable, 
 *  another modification is done, in order to open the ZED camera from the socket input, 
 *  by changing the init_paramters values in zed.open(init_parameters).
 */


/**********************************
 ** Using the ZED with Oculus Rift
 **********************************/

#define NOMINMAX

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <Windows.h>

#include <GL/glew.h>

#include <stddef.h>

/*Depending on the SDL version you are using, you may have to include SDL2/SDL.h or directly SDL.h (2.0.7)*/
#include <SDL.h>
#include <SDL_syswm.h>

#include <Extras/OVR_Math.h>
#include <OVR_CAPI.h>
#include <OVR_CAPI_GL.h>

#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include <sl/Camera.hpp>

#include "Shader.hpp"

#include <iostream>
#include <cstdlib>

using namespace std;

GLchar* OVR_ZED_VS =
"#version 330 core\n \
			layout(location=0) in vec3 in_vertex;\n \
			layout(location=1) in vec2 in_texCoord;\n \
			uniform uint isLeft; \n \
			out vec2 b_coordTexture; \n \
			void main()\n \
			{\n \
				if (isLeft == 1U)\n \
				{\n \
					b_coordTexture = in_texCoord;\n \
					gl_Position = vec4(in_vertex.x, in_vertex.y, in_vertex.z,1);\n \
				}\n \
				else \n \
				{\n \
					b_coordTexture = vec2(1.0 - in_texCoord.x, in_texCoord.y);\n \
					gl_Position = vec4(-in_vertex.x, in_vertex.y, in_vertex.z,1);\n \
				}\n \
			}";

GLchar* OVR_ZED_FS =
"#version 330 core\n \
			uniform sampler2D u_textureZED; \n \
			in vec2 b_coordTexture;\n \
			out vec4 out_color; \n \
			void main()\n \
			{\n \
				out_color = vec4(texture(u_textureZED, b_coordTexture).bgr,1); \n \
			}";

// Packed data for threaded computation
struct ThreadData {
    sl::Camera zed;
    sl::Mat zed_image[2];
    std::mutex mtx;
    bool run;
    bool new_frame;
};

vector< string> split(const string& s, char seperator) {
	vector< string> output;
	string::size_type prev_pos = 0, pos = 0;

	while ((pos = s.find(seperator, pos)) != string::npos) {
		string substring(s.substr(prev_pos, pos - prev_pos));
		output.push_back(substring);
		prev_pos = ++pos;
	}

	output.push_back(s.substr(prev_pos, pos - prev_pos));
	return output;
}

void setStreamParameter(sl::InitParameters& init_p, string& argument) {
	vector< string> configStream = split(argument, ':');
	sl::String ip(configStream.at(0).c_str());
	if (configStream.size() == 2) {
		init_p.input.setFromStream(ip, atoi(configStream.at(1).c_str()));
	}
	else init_p.input.setFromStream(ip);
}

// ZED image grab thread
void __zed_runner__(ThreadData &thread_data) {
    sl::RuntimeParameters runtime_parameters;
    runtime_parameters.enable_depth = false;
    // Set the ZED's CUDA context to this separate CPU thread
    cuCtxSetCurrent(thread_data.zed.getCUDAContext());
    // Loop while the main loop is not over
    while (thread_data.run) {
        // try to grab a new image
        if (thread_data.zed.grab(runtime_parameters) == sl::ERROR_CODE::SUCCESS) {
            // copy both left and right images
            thread_data.mtx.lock();
            thread_data.zed.retrieveImage(thread_data.zed_image[0], sl::VIEW::LEFT, sl::MEM::GPU);
            thread_data.zed.retrieveImage(thread_data.zed_image[1], sl::VIEW::RIGHT, sl::MEM::GPU);
            thread_data.mtx.unlock();
            thread_data.new_frame = true;
        } else
            sl::sleep_ms(2);
    }

    // Release the memory
    for (int eye = 0; eye < 2; eye++)
        thread_data.zed_image[eye].free();
    // Close the zed Camera
    thread_data.zed.close();
}


int main(int argc, char **argv) {
    // Initialize SDL2's context
    SDL_Init(SDL_INIT_VIDEO);
    // Initialize Oculus' context
    ovrResult result = ovr_Initialize(nullptr);
    if (OVR_FAILURE(result)) {
        std::cout << "ERROR: Failed to initialize libOVR" << std::endl;
        SDL_Quit();
        return -1;
    }

    ovrSession session;
    ovrGraphicsLuid luid;
    // Connect to the Oculus headset
    result = ovr_Create(&session, &luid);
    if (OVR_FAILURE(result)) {
        std::cout << "ERROR: Oculus Rift not detected" << std::endl;
        ovr_Shutdown();
        SDL_Quit();
        return -1;
    }

    int x = SDL_WINDOWPOS_CENTERED, y = SDL_WINDOWPOS_CENTERED;
    int winWidth = 1280;
    int winHeight = 720;
    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
    // Create SDL2 Window
    SDL_Window* window = SDL_CreateWindow("ZED Stereo Passthrough", x, y, winWidth, winHeight, flags);
    // Create OpenGL context
    SDL_GLContext glContext = SDL_GL_CreateContext(window);
    // Initialize GLEW
    glewInit();
    // Turn off vsync to let the compositor do its magic
    SDL_GL_SetSwapInterval(0);

    // Create a struct which contains the sl::Camera and the associated data
    ThreadData thread_data;

    // Initialize the ZED Camera
    sl::InitParameters init_parameters;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.camera_resolution = sl::RESOLUTION::HD720;

	// Set the camera input parameter as ip stream
	string stream_params;
	if (argc > 1) {
		stream_params = string(argv[1]);
	}
	else {
		cout << "\nOpening the stream requires the IP of the sender\n";
		cout << "Usage : ./ZED_Streaming_Receiver IP:[port]\n";
		cout << "You can specify it now, then press ENTER, 'IP:[port]': ";
		cin >> stream_params;
	}
	setStreamParameter(init_parameters, stream_params);

    sl::ERROR_CODE err_ = thread_data.zed.open(init_parameters);
    if (err_ != sl::ERROR_CODE::SUCCESS) {
        std::cout << "ERROR: " << sl::toString(err_) << std::endl;
        thread_data.zed.close();
        ovr_Destroy(session);
        ovr_Shutdown();
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }

    const auto& resolution = thread_data.zed.getCameraInformation().camera_configuration.resolution;
    auto zedWidth = static_cast<int>(resolution.width);
    auto zedHeight = static_cast<int>(resolution.height);

    sl::uchar4 dark_bckgrd(44, 44, 44, 255);
    GLuint zedTextureID[2];
    glGenTextures(2, zedTextureID);
    for (int eye = 0; eye < 2; eye++) {
        // Generate OpenGL texture
        glBindTexture(GL_TEXTURE_2D, zedTextureID[eye]);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, zedWidth, zedHeight, 0, GL_BGRA, GL_UNSIGNED_BYTE, NULL);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

        // Set size and default value to texture
        thread_data.zed_image[eye].alloc(zedWidth, zedHeight, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
        thread_data.zed_image[eye].setTo(dark_bckgrd, sl::MEM::GPU);
    }

    // Register texture
    cudaGraphicsResource *cimg_l;
    cudaGraphicsResource *cimg_r;
    cudaError_t  err = cudaGraphicsGLRegisterImage(&cimg_l, zedTextureID[ovrEye_Left], GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);
    cudaError_t  err2 = cudaGraphicsGLRegisterImage(&cimg_r, zedTextureID[ovrEye_Right], GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);
    if (err != cudaSuccess || err2 != cudaSuccess)
        std::cout << "ERROR: cannot create CUDA texture : " << err << std::endl;

    cuCtxSetCurrent(thread_data.zed.getCUDAContext());

    float pixel_density = 1.75f;
    ovrHmdDesc hmdDesc = ovr_GetHmdDesc(session);
	ovrInputState InputState;
    // Get the texture sizes of Oculus eyes
    ovrSizei textureSize0 = ovr_GetFovTextureSize(session, ovrEye_Left, hmdDesc.DefaultEyeFov[0], pixel_density);
    ovrSizei textureSize1 = ovr_GetFovTextureSize(session, ovrEye_Right, hmdDesc.DefaultEyeFov[1], pixel_density);
    // Compute the final size of the render buffer
    ovrSizei bufferSize;
    bufferSize.w = textureSize0.w + textureSize1.w;
    bufferSize.h = std::max(textureSize0.h, textureSize1.h);

    // Initialize OpenGL swap textures to render
    ovrTextureSwapChain textureChain = nullptr;
    // Description of the swap chain
    ovrTextureSwapChainDesc descTextureSwap = {};
    descTextureSwap.Type = ovrTexture_2D;
    descTextureSwap.ArraySize = 1;
    descTextureSwap.Width = bufferSize.w;
    descTextureSwap.Height = bufferSize.h;
    descTextureSwap.MipLevels = 1;
    descTextureSwap.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;
    descTextureSwap.SampleCount = 1;
    descTextureSwap.StaticImage = ovrFalse;
    // Create the OpenGL texture swap chain
    result = ovr_CreateTextureSwapChainGL(session, &descTextureSwap, &textureChain);

    ovrErrorInfo errInf;
    if (OVR_SUCCESS(result)) {
        int length = 0;
        ovr_GetTextureSwapChainLength(session, textureChain, &length);
        for (int i = 0; i < length; ++i) {
            GLuint chainTexId;
            ovr_GetTextureSwapChainBufferGL(session, textureChain, i, &chainTexId);
            glBindTexture(GL_TEXTURE_2D, chainTexId);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        }
    } else {
        ovr_GetLastErrorInfo(&errInf);
        std::cout << "ERROR: failed creating swap texture "<< errInf.ErrorString << std::endl;
        for (int eye = 0; eye < 2; eye++)
            thread_data.zed_image[eye].free();
        thread_data.zed.close();
        ovr_Destroy(session);
        ovr_Shutdown();
        SDL_GL_DeleteContext(glContext);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return -1;
    }
    // Generate frame buffer to render
    GLuint fboID;
    glGenFramebuffers(1, &fboID);
    // Generate depth buffer of the frame buffer
    GLuint depthBuffID;
    glGenTextures(1, &depthBuffID);
    glBindTexture(GL_TEXTURE_2D, depthBuffID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    GLenum internalFormat = GL_DEPTH_COMPONENT24;
    GLenum type = GL_UNSIGNED_INT;
    glTexImage2D(GL_TEXTURE_2D, 0, internalFormat, bufferSize.w, bufferSize.h, 0, GL_DEPTH_COMPONENT, type, NULL);

    // Create a mirror texture to display the render result in the SDL2 window
    ovrMirrorTextureDesc descMirrorTexture;
    memset(&descMirrorTexture, 0, sizeof(descMirrorTexture));
    descMirrorTexture.Width = winWidth;
    descMirrorTexture.Height = winHeight;
    descMirrorTexture.Format = OVR_FORMAT_R8G8B8A8_UNORM_SRGB;

    ovrMirrorTexture mirrorTexture = nullptr;
    result = ovr_CreateMirrorTextureGL(session, &descMirrorTexture, &mirrorTexture);
    if (!OVR_SUCCESS(result)) {
        ovr_GetLastErrorInfo(&errInf);
        std::cout << "ERROR: Failed to create mirror texture "<< errInf .ErrorString<< std::endl;
    }
    GLuint mirrorTextureId;
    ovr_GetMirrorTextureBufferGL(session, mirrorTexture, &mirrorTextureId);

    GLuint mirrorFBOID;
    glGenFramebuffers(1, &mirrorFBOID);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, mirrorTextureId, 0);
    glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    // Frame index used by the compositor, it needs to be updated each new frame
    long long frameIndex = 0;

    // FloorLevel will give tracking poses where the floor height is 0
    ovr_SetTrackingOriginType(session, ovrTrackingOrigin_FloorLevel);

    // Initialize a default Pose
    ovrPosef eyeRenderPose[2];
    ovrPosef hmdToEyeOffset[2];

    // Get the Oculus view scale description
    double sensorSampleTime;

    // Create and compile the shader's sources
    Shader shader(OVR_ZED_VS, OVR_ZED_FS);

    // Compute the useful part of the ZED image
    unsigned int widthFinal = bufferSize.w / 2;
    float heightGL = 1.f;
    float widthGL = 1.f;
    if (zedWidth > 0.f) {
        unsigned int heightFinal = zedHeight * widthFinal / (float) zedWidth;
        // Convert this size to OpenGL viewport's frame's coordinates
        heightGL = (heightFinal) / (float) (bufferSize.h);
        widthGL = ((zedWidth * (heightFinal / (float) zedHeight)) / (float) widthFinal);
    } else {
        std::cout << "WARNING: ZED parameters got wrong values."
            "Default vertical and horizontal FOV are used.\n"
            "Check your calibration file or check if your ZED is not too close to a surface or an object."
            << std::endl;
    }

    // Compute the Horizontal Oculus' field of view with its parameters
    float ovrFovH = (atanf(hmdDesc.DefaultEyeFov[0].LeftTan) + atanf(hmdDesc.DefaultEyeFov[0].RightTan));
    // Compute the Vertical Oculus' field of view with its parameters
    float ovrFovV = (atanf(hmdDesc.DefaultEyeFov[0].UpTan) + atanf(hmdDesc.DefaultEyeFov[0].DownTan));

    // Compute the center of the optical lenses of the headset
    float offsetLensCenterX = ((atanf(hmdDesc.DefaultEyeFov[0].LeftTan)) / ovrFovH) * 2.f - 1.f;
    float offsetLensCenterY = ((atanf(hmdDesc.DefaultEyeFov[0].UpTan)) / ovrFovV) * 2.f - 1.f;

    // Create a rectangle with the computed coordinates and push it in GPU memory
    struct GLScreenCoordinates {
        float left, up, right, down;
    } screenCoord;

    screenCoord.up = heightGL + offsetLensCenterY;
    screenCoord.down = heightGL - offsetLensCenterY;
    screenCoord.right = widthGL + offsetLensCenterX;
    screenCoord.left = widthGL - offsetLensCenterX;

    float rectVertices[12] = {-screenCoord.left, -screenCoord.up, 0, screenCoord.right, -screenCoord.up, 0, screenCoord.right, screenCoord.down, 0, -screenCoord.left, screenCoord.down, 0};
    GLuint rectVBO[3];
    glGenBuffers(1, &rectVBO[0]);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectVertices), rectVertices, GL_STATIC_DRAW);

    float rectTexCoord[8] = {0, 1, 1, 1, 1, 0, 0, 0};
    glGenBuffers(1, &rectVBO[1]);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
    glBufferData(GL_ARRAY_BUFFER, sizeof(rectTexCoord), rectTexCoord, GL_STATIC_DRAW);

    unsigned int rectIndices[6] = {0, 1, 2, 0, 2, 3};
    glGenBuffers(1, &rectVBO[2]);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(rectIndices), rectIndices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    // Initialize a boolean that will be used to stop the application�s loop and another one to pause/unpause rendering
    bool end = false;
    // SDL variable that will be used to store input events
    SDL_Event events;
    // This boolean is used to test if the application is focused
    bool isVisible = true;

    // Enable the shader
    glUseProgram(shader.getProgramId());
    // Bind the Vertex Buffer Objects of the rectangle that displays ZED images

    // vertices
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[0]);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
    // indices
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rectVBO[2]);
    // texture coordinates
    glEnableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
    glBindBuffer(GL_ARRAY_BUFFER, rectVBO[1]);
    glVertexAttribPointer(Shader::ATTRIB_TEXTURE2D_POS, 2, GL_FLOAT, GL_FALSE, 0, 0);

    // Set thread variables
    thread_data.run = true;
    thread_data.new_frame = true;
    // Launch ZED grab thread
    std::thread runner(__zed_runner__, std::ref(thread_data));

    cudaGraphicsMapResources(1, &cimg_l, 0);
    cudaGraphicsMapResources(1, &cimg_r, 0);

	// Execute python script
	printf("Executing python script...\n");
	char currentDir[MAX_PATH];
	GetCurrentDirectory(MAX_PATH, currentDir);
	printf(currentDir);
	const char* pythonScript = "\\..\\..\\scripts\\main.py";
	std::string command = "start cmd /k python " + std::string(currentDir) + std::string(pythonScript);
	if (system(command.c_str()) != 0) {
		std::cerr << "Error executing Python script" << std::endl;
		return 1;
	}

	// Create named pipe
	LPCSTR pipeName = "\\\\.\\pipe\\MyPipe";
	HANDLE hPipe = CreateNamedPipe(
		pipeName,
		PIPE_ACCESS_OUTBOUND,
		PIPE_TYPE_BYTE | PIPE_READMODE_BYTE | PIPE_WAIT,
		1,
		0,
		0,
		0,
		NULL
	);

	if (hPipe == INVALID_HANDLE_VALUE) {
		std::cerr << "Error creating named pipe: " << GetLastError() << std::endl;
		return 1;
	}

	// Connect to the named pipe
	if (ConnectNamedPipe(hPipe, NULL) == FALSE) {
		std::cerr << "Error connecting to named pipe: " << GetLastError() << std::endl;
		CloseHandle(hPipe);
		return 1;
	}


    // Main loop
    while (!end) {

        // While there is an event catched and not tested
        while (SDL_PollEvent(&events)) {
            // If a key is released
            if (events.type == SDL_KEYUP) {
                // If Q -> quit the application
                if (events.key.keysym.scancode == SDL_SCANCODE_Q)
                    end = true;
            }
        }

        // Get texture swap index where we must draw our frame
        GLuint curTexId;
        int curIndex;
        ovr_GetTextureSwapChainCurrentIndex(session, textureChain, &curIndex);
        ovr_GetTextureSwapChainBufferGL(session, textureChain, curIndex, &curTexId);

        // Call ovr_GetRenderDesc each frame to get the ovrEyeRenderDesc, as the returned values (e.g. HmdToEyeOffset) may change at runtime.
        hmdToEyeOffset[ovrEye_Left] = ovr_GetRenderDesc(session, ovrEye_Left, hmdDesc.DefaultEyeFov[ovrEye_Left]).HmdToEyePose;
        hmdToEyeOffset[ovrEye_Right] = ovr_GetRenderDesc(session, ovrEye_Right, hmdDesc.DefaultEyeFov[ovrEye_Right]).HmdToEyePose;

        // Get eye poses, feeding in correct IPD offset
        ovr_GetEyePoses2(session, frameIndex, ovrTrue, hmdToEyeOffset, eyeRenderPose, &sensorSampleTime);

        // If the application is focused
        if (isVisible) {
            // If successful grab a new ZED image
            if (thread_data.new_frame) {
                // Update the ZED frame counter
                thread_data.mtx.lock();
                cudaArray_t arrIm;
                cudaGraphicsSubResourceGetMappedArray(&arrIm, cimg_l, 0, 0);
                cudaMemcpy2DToArray(arrIm, 0, 0, thread_data.zed_image[ovrEye_Left].getPtr<sl::uchar1>(sl::MEM::GPU), thread_data.zed_image[ovrEye_Left].getStepBytes(sl::MEM::GPU), thread_data.zed_image[ovrEye_Left].getWidth() * 4, thread_data.zed_image[ovrEye_Left].getHeight(), cudaMemcpyDeviceToDevice);

                cudaGraphicsSubResourceGetMappedArray(&arrIm, cimg_r, 0, 0);
                cudaMemcpy2DToArray(arrIm, 0, 0, thread_data.zed_image[ovrEye_Right].getPtr<sl::uchar1>(sl::MEM::GPU), thread_data.zed_image[ovrEye_Right].getStepBytes(sl::MEM::GPU), thread_data.zed_image[ovrEye_Left].getWidth() * 4, thread_data.zed_image[ovrEye_Left].getHeight(), cudaMemcpyDeviceToDevice);
                thread_data.mtx.unlock();
                thread_data.new_frame = false;

                // Bind the frame buffer
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                // Set its color layer 0 as the current swap texture
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, curTexId, 0);
                // Set its depth layer as our depth buffer
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depthBuffID, 0);
                // Clear the frame buffer
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glClearColor(0, 0, 0, 1);

                // Render for each Oculus eye the equivalent ZED image
                for (int eye = 0; eye < 2; eye++) {
                    // Set the left or right vertical half of the buffer as the viewport
                    glViewport(eye == ovrEye_Left ? 0 : bufferSize.w / 2, 0, bufferSize.w / 2, bufferSize.h);
                    // Bind the left or right ZED image
                    glBindTexture(GL_TEXTURE_2D, eye == ovrEye_Left ? zedTextureID[ovrEye_Left] : zedTextureID[ovrEye_Right]);
                    // Bind the isLeft value
                    glUniform1ui(glGetUniformLocation(shader.getProgramId(), "isLeft"), eye == ovrEye_Left ? 1U : 0U);
                    // Draw the ZED image
                    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
                }
                // Avoids an error when calling SetAndClearRenderSurface during next iteration.
                // Without this, during the next while loop iteration SetAndClearRenderSurface
                // would bind a framebuffer with an invalid COLOR_ATTACHMENT0 because the texture ID
                // associated with COLOR_ATTACHMENT0 had been unlocked by calling wglDXUnlockObjectsNV.
                glBindFramebuffer(GL_FRAMEBUFFER, fboID);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
                glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, 0, 0);
                // Commit changes to the textures so they get picked up frame
                ovr_CommitTextureSwapChain(session, textureChain);
            }
            // Do not forget to increment the frameIndex!
            frameIndex++;
        }

        /*
        Note: Even if we don't ask to refresh the framebuffer or if the Camera::grab()
              doesn't catch a new frame, we have to submit an image to the Rift; it
                  needs 75Hz refresh. Else there will be jumbs, black frames and/or glitches
                  in the headset.
         */
        ovrLayerEyeFov ld;
        ld.Header.Type = ovrLayerType_EyeFov;
        // Tell to the Oculus compositor that our texture origin is at the bottom left
        ld.Header.Flags = ovrLayerFlag_TextureOriginAtBottomLeft; // Because OpenGL | Disable head tracking
        // Set the Oculus layer eye field of view for each view
        for (int eye = 0; eye < 2; ++eye) {
            // Set the color texture as the current swap texture
            ld.ColorTexture[eye] = textureChain;
            // Set the viewport as the right or left vertical half part of the color texture
            ld.Viewport[eye] = OVR::Recti(eye == ovrEye_Left ? 0 : (bufferSize.w / 2) + 200, 0, (bufferSize.w / 2) - 200, bufferSize.h);
            // Set the field of view
            ld.Fov[eye] = hmdDesc.DefaultEyeFov[eye];
            // Set the pose matrix
            ld.RenderPose[eye] = eyeRenderPose[eye];
        }

        ld.SensorSampleTime = sensorSampleTime;

        ovrLayerHeader* layers = &ld.Header;
        // Submit the frame to the Oculus compositor
        // which will display the frame in the Oculus headset
        result = ovr_SubmitFrame(session, frameIndex, nullptr, &layers, 1);

        if (!OVR_SUCCESS(result)) {
            ovr_GetLastErrorInfo(&errInf);
            std::cout << "ERROR: failed to submit frame "<< errInf.ErrorString << std::endl;
            end = true;
        }

        if (result == ovrSuccess && !isVisible) {
            std::cout << "The application is now shown in the headset." << std::endl;
        }
        isVisible = (result == ovrSuccess);

        // This is not really needed for this application but it may be useful for an more advanced application
        ovrSessionStatus sessionStatus;
        ovr_GetSessionStatus(session, &sessionStatus);
        if (sessionStatus.ShouldRecenter) {
            std::cout << "Recenter Tracking asked by Session" << std::endl;
            ovr_RecenterTrackingOrigin(session);
        }

        // Copy the frame to the mirror buffer
        // which will be drawn in the SDL2 image
        glBindFramebuffer(GL_READ_FRAMEBUFFER, mirrorFBOID);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
        GLint w = winWidth;
        GLint h = winHeight;
        glBlitFramebuffer(0, h, w, 0,
                          0, 0, w, h,
                          GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
        // Swap the SDL2 window
        SDL_GL_SwapWindow(window);

		// Query the HMD for ts current tracking state.
		ovrTrackingState ts = ovr_GetTrackingState(session, ovr_GetTimeInSeconds(), ovrTrue);
		ovr_GetInputState(session, ovrControllerType_Touch, &InputState);
		ovrVector2f rightStick = InputState.Thumbstick[ovrHand_Right];
		ovrVector2f leftStick = InputState.Thumbstick[ovrHand_Left];
		const float radialDeadZone = 0.5;
		if (std::abs(rightStick.x) < radialDeadZone) rightStick.x = 0.0;
		if (std::abs(rightStick.y) < radialDeadZone) rightStick.y = 0.0;

		printf(
			" Touch Lin Vel (XY): %4.2f  %4.2f\n"
			" Touch Rot Vel (Z):  %4.2f\n",
			" HMD Ang Vel (YPR): %4.2f  %4.2f  %4.2f\n",
			rightStick.y, rightStick.x,
			leftStick.x,
			ts.HeadPose.AngularVelocity.y, -ts.HeadPose.AngularVelocity.x, -ts.HeadPose.AngularVelocity.z);

		// Send float data
		float data[] = {ts.HeadPose.AngularVelocity.y, -ts.HeadPose.AngularVelocity.x, -ts.HeadPose.AngularVelocity.z, rightStick.y, rightStick.x, leftStick.x};

		DWORD bytesWritten;
		if (WriteFile(hPipe, data, sizeof(data), &bytesWritten, NULL) == FALSE) {
			std::wcerr << "Error writing to named pipe: " << GetLastError() << std::endl;
		}
    }

	// Close the pipe
	CloseHandle(hPipe);

    cudaGraphicsUnmapResources(1, &cimg_l);
    cudaGraphicsUnmapResources(1, &cimg_r);
    thread_data.run = false;

    // Disable all OpenGL buffer
    glDisableVertexAttribArray(Shader::ATTRIB_TEXTURE2D_POS);
    glDisableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
    glBindVertexArray(0);
    // Delete the Vertex Buffer Objects of the rectangle
    glDeleteBuffers(3, rectVBO);
    // Delete SDL, OpenGL, Oculus and ZED context
    ovr_DestroyTextureSwapChain(session, textureChain);
    ovr_DestroyMirrorTexture(session, mirrorTexture);
    ovr_Destroy(session);
    ovr_Shutdown();
    SDL_GL_DeleteContext(glContext);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    runner.join();

    // Quit
    return 0;
}
