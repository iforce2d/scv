
// Visualization for scv planner using Dear ImGui, OpenGL2 implementation.

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"
#include <stdio.h>
#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <GL/glu.h>

#include <chrono>
#include "implot.h"
#include "planner.h"
#include "camera.h"

using namespace std;
using namespace scv;

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

GLFWwindow* window;

Camera camera;      // handles a FPS game style input (WASD, left shift, left ctrl)
planner plan;       // the S-curve planner we're testing

// These are to cycle between red/green/blue when drawing segments to differentiate them
vec3 colors[] = {
    vec3(0.9f, 0.3f, 0.3f),
    vec3(0.3f, 0.9f, 0.3f),
    vec3(0.3f, 0.3f, 0.9f)
};

// Draw 3 simple lines to show the world origin location
void drawAxes() {

    glLineWidth(3);
    glBegin(GL_LINES);

    glColor3f(1,0,0);
    glVertex3f(0,0,0);
    glVertex3f(1,0,0);

    glColor3f(0,1,0);
    glVertex3f(0,0,0);
    glVertex3f(0,1,0);

    glColor3f(0,0,1);
    glVertex3f(0,0,0);
    glVertex3f(0,0,1);

    glEnd();
}

// Draw a simple bounding box to show the position constraints of the planner
void drawPlannerBoundingBox() {
    glLineWidth(1);
    glColor3f(0,0.66,0);
    glBegin(GL_LINE_LOOP);
    glVertex3d( plan.posLimitLower.x, plan.posLimitLower.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitLower.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitUpper.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitUpper.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitUpper.y, plan.posLimitUpper.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitUpper.y, plan.posLimitUpper.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitLower.y, plan.posLimitUpper.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitLower.y, plan.posLimitUpper.z );
    glEnd();
    glBegin(GL_LINES);
    glVertex3d( plan.posLimitUpper.x, plan.posLimitLower.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitLower.y, plan.posLimitUpper.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitUpper.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitUpper.x, plan.posLimitUpper.y, plan.posLimitUpper.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitLower.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitUpper.y, plan.posLimitLower.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitLower.y, plan.posLimitUpper.z );
    glVertex3d( plan.posLimitLower.x, plan.posLimitUpper.y, plan.posLimitUpper.z );
    glEnd();
}

// These arrays will be used to draw the plots
#define MAXPLOTPOINTS 2048
float plotTime[MAXPLOTPOINTS]; // x axis of the plot, all others are y-axis values
float plotPosX[MAXPLOTPOINTS], plotPosY[MAXPLOTPOINTS], plotPosZ[MAXPLOTPOINTS];
float plotVelX[MAXPLOTPOINTS], plotVelY[MAXPLOTPOINTS], plotVelZ[MAXPLOTPOINTS];
float plotAccX[MAXPLOTPOINTS], plotAccY[MAXPLOTPOINTS], plotAccZ[MAXPLOTPOINTS];
float plotVelMag[MAXPLOTPOINTS], plotAccMag[MAXPLOTPOINTS], plotJerkMag[MAXPLOTPOINTS];
int numPlotPoints = 0; // how many plot points are actually filled

// These are used to get a moving average of the time taken to calculate the full trajectory
#define NUMCALCTIMES 64
float calcTimes[NUMCALCTIMES];
float calcTimeTotal = 0;
int calcTimeInd = 0;
float calcTime = 0; // final result to show in GUI

float animAdvance = 0;  // used to animate a white dot moving along the path
vec3 animLoc;
bool showBoundingBox = true;
bool showControlPoints = true;

// This function will be called during ImGui's rendering, while drawing the background.
// Draw all our stuff here so the GUI windows will then be over the top of it. Need to
// re-enable scissor test after we're done.
void backgroundRenderCallback(const ImDrawList* parent_list, const ImDrawCmd* cmd) {

    // calculate the trajectory every frame, and measure the time taken
    std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    plan.calculateMoves();
    std::chrono::steady_clock::time_point t1 =   std::chrono::steady_clock::now();

    // update the moving average
    long int timeTaken = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
    calcTimeTotal -= calcTimes[calcTimeInd];
    calcTimeTotal += timeTaken;
    calcTimes[calcTimeInd] = timeTaken;
    calcTimeInd = (calcTimeInd + 1) % NUMCALCTIMES;
    calcTime = 0;
    for (int i = 0; i < NUMCALCTIMES; i++) {
        calcTime += calcTimes[i];
    }
    calcTime /= NUMCALCTIMES;

    // apply view transforms
    int w, h;
    glfwGetFramebufferSize(window, &w, &h);
    float aspectRatio = w / (float)h;

    glDisable(GL_SCISSOR_TEST);

    glClearColor(0,0,0,0);
    glClear(GL_COLOR_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluPerspective( 45, aspectRatio, 1, 100 );

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    camera.gluLookAt();

    drawAxes();

    if ( showBoundingBox ) {
        drawPlannerBoundingBox();
    }


//    size_t segmentIndex = 0;
//    float timeBase = 0;
//    int tCount = 0;
//    float timeStepsOverFlow = 0;
//    float timePerDivision = 0.01;

    numPlotPoints = 0;

    glPointSize(4);
    glBegin(GL_POINTS);

    vec3 p, v, a, j; // position, velocity, acceleration, jerk
    double t = 0;
    int count = 0;
    int segmentIndex;
    while ( true ) {

        bool stillOnPath = plan.getTrajectoryState(t, &segmentIndex, &p, &v, &a, &j);

        vec3 c = colors[segmentIndex%3];
        glColor3f(c.x,c.y,c.z);
        glVertex3f( p.x, p.y, p.z );

        if ( count < MAXPLOTPOINTS ) {
            plotTime[count] = t;
            plotPosX[count] = p.x;
            plotPosY[count] = p.y;
            plotPosZ[count] = p.z;
            plotVelX[count] = v.x;
            plotVelY[count] = v.y;
            plotVelZ[count] = v.z;
            plotAccX[count] = a.x;
            plotAccY[count] = a.y;
            plotAccZ[count] = a.z;
            plotVelMag[count] = v.Length();
            plotAccMag[count] = a.Length();
            plotJerkMag[count] = j.Length();
        }

        t += 0.01;
        count++;

        if ( ! stillOnPath )
            break;
    }

    /*vector<segment>& segments = plan.getSegments();
    while (segmentIndex < segments.size()) {

        scv::segment& s = segments[segmentIndex];

        vec3 c = colors[segmentIndex%3];
        glColor3f(c.x,c.y,c.z);

        float localT = timeStepsOverFlow * timePerDivision;

        while ( localT < s.duration ) {

            float globalT = tCount * timePerDivision;

            scv::vec3 p, v, a, j;
            plan.getScurvePoint(s, localT, &p, &v, &a, &j);

            if ( tCount < MAXPLOTPOINTS ) {
                plotTime[tCount] = globalT;
                plotPosX[tCount] = p.x;
                plotPosY[tCount] = p.y;
                plotPosZ[tCount] = p.z;
                plotVelX[tCount] = v.x;
                plotVelY[tCount] = v.y;
                plotVelZ[tCount] = v.z;
                plotAccX[tCount] = a.x;
                plotAccY[tCount] = a.y;
                plotAccZ[tCount] = a.z;

                plotVelMag[tCount] = v.Length();
                plotAccMag[tCount] = a.Length();
                plotJerkMag[tCount] = j.Length();
            }

            glVertex3f( p.x, p.y, p.z );

            localT += timePerDivision;
            tCount++;
        }

        timeStepsOverFlow = (localT - s.duration) / timePerDivision;
        segmentIndex++;
        timeBase += 1;
    }*/

    glEnd();

    numPlotPoints = scv::min(count, MAXPLOTPOINTS);

    if ( showControlPoints ) {
        glPointSize(8);
        glColor3f(1,0,1);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < plan.moves.size(); i++) {
            scv::move& m = plan.moves[i];
            if ( i == 0 )
                glVertex3d( m.src.x, m.src.y, m.src.z );
            glVertex3d( m.dst.x, m.dst.y, m.dst.z );
        }
        glEnd();
    }

    bool animRunning = plan.advanceTraverse( animAdvance, &animLoc );

    glPointSize(12);
    if ( animRunning )
        glColor3f(1,1,1);
    else
        glColor3f(0.5,0.5,0.5);
    glBegin(GL_POINTS);
    glVertex3d( animLoc.x, animLoc.y, animLoc.z );
    glEnd();

    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();

    glEnable(GL_SCISSOR_TEST);
}

// A convenience function to show vec3 components as individual inputs
void showVec3Editor(const char* label, vec3 *v) {
    char n[128];
    sprintf(n, "%s X", label);
    ImGui::InputDouble(n, &v->x, 0.1f, 1.0f);
    sprintf(n, "%s Y", label);
    ImGui::InputDouble(n, &v->y, 0.1f, 1.0f);
    sprintf(n, "%s Z", label);
    ImGui::InputDouble(n, &v->z, 0.1f, 1.0f);
}

void showPlots() {
    if (ImPlot::BeginPlot("Line Plots")) {
        ImPlot::SetupAxes("t","");
        ImPlot::PlotLine("Pos X", plotTime, plotPosX, numPlotPoints);
        ImPlot::PlotLine("Pos Y", plotTime, plotPosY, numPlotPoints);
        ImPlot::PlotLine("Pos Z", plotTime, plotPosZ, numPlotPoints);
        ImPlot::PlotLine("Vel X", plotTime, plotVelX, numPlotPoints);
        ImPlot::PlotLine("Vel Y", plotTime, plotVelY, numPlotPoints);
        ImPlot::PlotLine("Vel Z", plotTime, plotVelZ, numPlotPoints);
        ImPlot::PlotLine("Acc X", plotTime, plotAccX, numPlotPoints);
        ImPlot::PlotLine("Acc Y", plotTime, plotAccY, numPlotPoints);
        ImPlot::PlotLine("Acc Z", plotTime, plotAccZ, numPlotPoints);
        ImPlot::PlotLine("Vel mag", plotTime, plotVelMag, numPlotPoints);
        ImPlot::PlotLine("Acc mag", plotTime, plotAccMag, numPlotPoints);
        ImPlot::PlotLine("Jerk mag", plotTime, plotJerkMag, numPlotPoints);
        ImPlot::EndPlot();
    }
}

void randomizePoints() {

    scv::vec3 lastRandPos;

    for (size_t i = 0; i < plan.moves.size(); i++) {
        scv::move& m = plan.moves[i];

        if ( i > 0 )
            m.src = lastRandPos;

        scv::vec3 r;

        if ( i == 0 ) {
            r = scv::vec3(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
            r = r * plan.posLimitUpper;
            r += plan.posLimitLower;
            m.src = r;
        }

        r = scv::vec3(rand() / (float)RAND_MAX, rand() / (float)RAND_MAX, rand() / (float)RAND_MAX);
        r = r * plan.posLimitUpper;
        r += plan.posLimitLower;
        m.dst = r;

        lastRandPos = r;

    }
}








static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

int main(int, char**)
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    window = glfwCreateWindow(1280, 720, "S-Curve visualizer", nullptr, nullptr);
    if (window == nullptr)
        return 1;

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL2_Init();
    ImGui::StyleColorsDark();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    //io.Fonts->AddFontFromFileTTF("/usr/share/fonts/gnu-free/FreeSans.ttf", 16.0f);

    camera.setLocation(-5, -10, 10);
    camera.setDirection( 28, -25 );

    plan.setPositionLimits(0, 0, 0, 10, 10, 7);
    plan.setVelocityLimits(200, 200, 200);
    plan.setAccelerationLimits(1000, 1000, 1000);
    plan.setJerkLimits(2000, 2000, 2000);

    scv::move m;
    m.vel = 12;
    m.acc = 400;
    m.jerk = 800;
    m.blendType = CBT_MAX_JERK;
    m.src = vec3( 1, 1, 0);
    m.dst = vec3( 1, 1, 6);    plan.appendMove(m);
    m.dst = vec3( 1, 9, 6);    plan.appendMove(m);
    m.dst = vec3( 1, 9, 0);    plan.appendMove(m);
    m.dst = vec3( 5, 9, 0);    plan.appendMove(m);
    m.dst = vec3( 5, 5, 0);    plan.appendMove(m);
    m.dst = vec3( 5, 1, 6);    plan.appendMove(m);
    m.dst = vec3( 9, 1, 6);    plan.appendMove(m);
    m.dst = vec3( 9, 1, 0);    plan.appendMove(m);
    m.dst = vec3( 9, 9, 0);    plan.appendMove(m);
    m.dst = vec3( 9, 9, 6);    plan.appendMove(m);

    plan.calculateMoves();
    //plan.printConstraints();
    //plan.printMoves();
    //plan.printSegments();

    //bool show_demo_window = false;
    bool doRandomizePoints = false;
    float animSpeedScale = 1;

    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        if ( glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS )
            break;

        if ( io.Framerate != 0 ) // framerate will be zero on first frame
            animAdvance = 1 / io.Framerate * animSpeedScale;

        if ( doRandomizePoints ) {
            randomizePoints();
        }

        if ( ImGui::IsMouseDown(1) ) {// hold right mouse button to pan view
            float yaw = camera.yaw + io.MouseDelta.x * 0.05;
            float pitch = camera.pitch + io.MouseDelta.y * -0.05;
            camera.setDirection(yaw, pitch);
        }

        float camMoveSpeed = 0.15;

        float right = 0;
        float forward = 0;
        float up = 0;
        if ( ImGui::IsKeyDown(ImGuiKey_A) ) {
            right = -camMoveSpeed;
        }
        else if ( ImGui::IsKeyDown(ImGuiKey_D) ) {
            right = camMoveSpeed;
        }        

        if ( ImGui::IsKeyDown(ImGuiKey_S) ) {
            forward = -camMoveSpeed;
        }
        else if ( ImGui::IsKeyDown(ImGuiKey_W) ) {
            forward = camMoveSpeed;
        }

        if ( ImGui::IsKeyDown(ImGuiKey_LeftCtrl) ) {
            up = -camMoveSpeed;
        }
        else if ( ImGui::IsKeyDown(ImGuiKey_LeftShift) ) {
            up = camMoveSpeed;
        }
        camera.translate(right, forward, up);

        if ( ImGui::IsKeyDown(ImGuiKey_T) ) {
            plan.resetTraverse();
        }


        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImDrawList* bgdl = ImGui::GetBackgroundDrawList();
        bgdl->AddCallback(backgroundRenderCallback, nullptr);

        //if (show_demo_window)
        //    ImGui::ShowDemoWindow(&show_demo_window);

        {

            ImGui::Begin("Settings");

            //ImGui::Checkbox("Demo Window", &show_demo_window);      // Edit bools storing our window open/close state

            //showVec3Editor("animLoc", &animLoc);

            char n[128];

            if (ImGui::CollapsingHeader("Display options"))
            {
                if (ImGui::BeginTable("split", 3))
                {
                    ImGui::TableNextColumn(); ImGui::Checkbox("Show bounding box", &showBoundingBox);
                    ImGui::TableNextColumn(); ImGui::Checkbox("Show control points", &showControlPoints);
                    ImGui::EndTable();
                }

                ImGui::SliderFloat("Animation speed", &animSpeedScale, 0, 5);
            }

            if (ImGui::CollapsingHeader("Planner settings"))
            {
                ImGui::SeparatorText("Position constraint");
                showVec3Editor("Pos", &plan.posLimitUpper);

                ImGui::SeparatorText("Velocity constraint");
                showVec3Editor("Vel", &plan.velLimit);

                ImGui::SeparatorText("Acceleration constraint");
                showVec3Editor("Acc", &plan.accLimit);

                ImGui::SeparatorText("Jerk constraint");
                showVec3Editor("Jerk", &plan.jerkLimit);
            }

            if (ImGui::CollapsingHeader("Control points"))
            {
                ImGui::Checkbox("Randomize", &doRandomizePoints);

                if ( ! plan.moves.empty() ) {
                    scv::move& m = plan.moves[0];
                    if (ImGui::TreeNode("Point 0")) {

                        ImGui::SeparatorText("Location");
                        showVec3Editor("Loc 0", &m.src);

                        ImGui::TreePop();
                    }
                }

                for (size_t i = 0; i < plan.moves.size(); i++) {
                    scv::move& m = plan.moves[i];
                    sprintf(n, "Point %d", (int)(i+1));
                    if (ImGui::TreeNode(n)) {

                        ImGui::SeparatorText("Location");
                        sprintf(n, "Loc %d", (int)(i+1));
                        showVec3Editor(n, &m.dst);

                        ImGui::SeparatorText("Constraints");
                        sprintf(n, "Vel %d", (int)(i+1));
                        ImGui::InputDouble(n, &m.vel, 0.1f, 1.0f);
                        sprintf(n, "Acc %d", (int)(i+1));
                        ImGui::InputDouble(n, &m.acc, 0.1f, 1.0f);
                        sprintf(n, "Jerk %d", (int)(i+1));
                        ImGui::InputDouble(n, &m.jerk, 0.1f, 1.0f);

                        if ( i > 0 ) {
                            int e = m.blendType;
                            ImGui::RadioButton("None", &e, CBT_NONE); ImGui::SameLine();
                            ImGui::RadioButton("Min jerk", &e, CBT_MIN_JERK); ImGui::SameLine();
                            ImGui::RadioButton("Max jerk", &e, CBT_MAX_JERK);
                            m.blendType = (cornerBlendType)e;
                        }

                        ImGui::TreePop();
                    }
                }

                for (size_t i = 1; i < plan.moves.size(); i++) {
                    scv::move& m = plan.moves[i];
                    scv::move& prevMove = plan.moves[i-1];
                    m.src.x = prevMove.dst.x;
                    m.src.y = prevMove.dst.y;
                    m.src.z = prevMove.dst.z;
                }

                if (ImGui::Button("Add point")) {
                    vec3 p = vec3_zero;
                    if ( ! plan.moves.empty() ) {
                        scv::move& el = plan.moves[ plan.moves.size()-1 ];
                        p = el.dst;
                    }
                    scv::move m;
                    m.src = p;
                    m.dst = p + vec3( 2, 2, 2 );
                    m.vel = 12;
                    m.acc = 400;
                    m.jerk = 800;
                    plan.appendMove(m);
                }

            }

            ImGui::Text("Framerate average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::Text("SCV calculation time %.1f us", calcTime);

            showPlots();

            ImGui::End();
        }

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwMakeContextCurrent(window);
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
