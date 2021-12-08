#include "SDLViewer.h"

#include <Eigen/Core>

#include <functional>
#include <iostream>
#include <thread>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!

#include "stb_image_write.h"
#include "attributes.h"
//#include <chrono>
#include <vector>



int find_triangle(std::vector<Triangle> &triangles, Eigen::Vector4f &point) {
    for (int i=0; i < triangles.size(); i++) {
        auto &triangle = triangles[i];
        if (triangle.isInside(point)) {
            return i;
        }
    }
    return -1;
}


UniformAttributes::UniformAttributes() {
    view_Matri = Eigen::Matrix<float,4,4>::Identity();
    inver_Matri = Eigen::Matrix<float,4,4>::Identity();
} 

void inver_view(UniformAttributes &uniform, float &nx, float &ny) {
    Eigen::Vector4f vec(nx, ny, 0, 1);
    vec= {vec.x() / float(uniform.width) * 2 - 1, (float(uniform.height)-vec.y())/float(uniform.height)*2 -1 , 0, 1};
    vec = uniform.inver_Matri*vec;
    vec = {0.5*(1+vec.x())*float(uniform.width), float(uniform.height) - 0.5*(1+vec.y())*float(uniform.height), 0, 1};
    nx = vec.x();
    ny = vec.y();
}


void zoom_matrix(UniformAttributes &uniform, int zoomIn) {
    float factor;
    if(zoomIn > 0){
        factor =1.2;
    }
    else{
        factor =0.8;
    }
    
    Eigen::Matrix<float, 4, 4> m;
    m << factor,0,0,0,
        0,factor,0,0,
        0,0,1,0,
        0,0,0,1;
    Eigen::Matrix<float, 4, 4> inverse;
    inverse << 1/factor,0,0,0 ,
    0,1/factor,0,0 ,
    0,0,1,0 ,
    0,0,0,1;
    uniform.view_Matri = m*uniform.view_Matri;
    uniform.inver_Matri = uniform.inver_Matri*inverse;

}

void translate_matrix(UniformAttributes &uniform, float offsetX, float offsetY) {
    Eigen::Matrix<float, 4, 4> m;
    m << 1,0,0,offsetX,
            0,1,0,offsetY,
            0,0,1,0 ,
            0,0,0,1;
    Eigen::Matrix<float, 4, 4> inverse;
    inverse << 1,0,0,-offsetX ,
            0,1,0,-offsetY,
            0,0,1,0 ,
            0,0,0,1;
    uniform.view_Matri = m*uniform.view_Matri;
    uniform.inver_Matri = uniform.inver_Matri*inverse;
}


void draw(Program &program, FrameBuffer& frameBuffer, SDLViewer &viewer, std::vector<Triangle> &triangles) {
    for (unsigned i = 0; i < frameBuffer.rows(); i++) {
        for (unsigned j = 0; j < frameBuffer.cols(); j++) {
            frameBuffer(i, j).color << 255, 255, 255, 255;
        }
    }
    std::vector<VertexAttributes>  triangleVertices;
    triangleVertices.reserve(triangles.size() * 3);
    for (auto &triangle : triangles) {
        for (auto &v: triangle.vs) {
            VertexAttributes temp = v;
            if (triangle.highlight) {
                temp.color.x() = 1 - temp.color.x();
                temp.color.y() = 1 - temp.color.y();
                temp.color.z() = 1 - temp.color.z();
            }
             triangleVertices.push_back(temp);
        }
    }
    std::vector<VertexAttributes> lineVertices;
    lineVertices.reserve(triangles.size() * 6 + 6); 
    for (auto &triangle : triangles) {
        for (int i = 0; i < 3; i++) {
            lineVertices.push_back(triangle.vs[i]);
            lineVertices.push_back(triangle.vs[(i+1)%3]);
        }
    }
    auto &uniform = viewer.uniform;
    int width = uniform.width;
    int height = uniform.height;
    if (viewer.insertionBuffer.size() == 3) {
        lineVertices.push_back(viewer.insertionBuffer[0]);
        lineVertices.push_back(viewer.insertionBuffer[1]);
        lineVertices.push_back(viewer.insertionBuffer[1]);
        lineVertices.push_back(viewer.insertionBuffer[2]);
        lineVertices.push_back(viewer.insertionBuffer[2]);
        lineVertices.push_back(viewer.insertionBuffer[0]);
    } else if (viewer.insertionBuffer.size() == 2) {
        lineVertices.push_back(viewer.insertionBuffer[0]);
        lineVertices.push_back(viewer.insertionBuffer[1]);
    }
    for (auto &v: lineVertices) {
        v.color << 0,0,0,1;
    }
    rasterize_lines(program, uniform, lineVertices, 1, frameBuffer);
    rasterize_triangles(program, uniform, triangleVertices, frameBuffer);

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

    for (unsigned i = 0; i < frameBuffer.rows(); i++) {
        for (unsigned j = 0; j < frameBuffer.cols(); j++) {
            R(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(0);
            G(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(1);
            B(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(2);
            A(i, frameBuffer.cols() - 1 - j) = frameBuffer(i, j).color(3);
        }
    }
    viewer.draw_image(R, G, B, A);
}

std::function<void(SDLViewer &)> linear_ani(Program &program, FrameBuffer& frameBuffer) {
    return [&](SDLViewer &viewer) {
        if (!viewer.animate) {
            return;
        }
        viewer.animate = false;
        int middleFramesNum = 1000;
        std::vector<std::vector<Triangle>> frames((middleFramesNum) * (viewer.keyframes.size()-1)+1, std::vector<Triangle>());
        for (int i = 0; i < viewer.keyframes.size()-1; i++) {
            auto &startFrame = viewer.keyframes[i];
            auto &endFrame = viewer.keyframes[i+1];
            for (int j = 0; j < startFrame.size(); j++) {
                auto &t1 = startFrame[j];
                auto &t2 = endFrame[j];
                for (int p = 0; p < middleFramesNum; p++) {
                    Triangle middle = t1;
                    float t = float(p)/float(middleFramesNum);
                    for (int k = 0; k < 3; k++) {
                        middle.vs[k].position.x() = t1.vs[k].position.x()+(t2.vs[k].position.x()-t1.vs[k].position.x())*t;
                        middle.vs[k].position.y() = t1.vs[k].position.y()+(t2.vs[k].position.y()-t1.vs[k].position.y())*t;
                    }
                    frames[i*middleFramesNum+p].push_back(middle);
                }
            }
        }
        for (auto &t: viewer.keyframes.back()) {
            frames.back().push_back(t);
        }

        for (auto &f: frames) {
            draw(program, frameBuffer, viewer, f);
        }

    };
}

Eigen::Vector4f bezier_re(const std::vector<Eigen::Vector4f> &current, float t) {
    if (current.size() == 1) {
        return current.front();
    }
    std::vector<Eigen::Vector4f> next;
    next.reserve(current.size()-1);
    for (int i = 0 ; i < current.size()-1; i++) {
        next.emplace_back(current[i]*(1-t) + current[i+1]*t);
    }
    return bezier_re(next, t);
}

std::function<void(SDLViewer &)> bezier_ani(Program &program, FrameBuffer& frameBuffer) {
    return [&](SDLViewer &viewer) {
        if (!viewer.animate) {
            return;
        }
        viewer.animate = false;
        int middleFramesNum = 500*viewer.keyframes.size();
        std::vector<std::vector<Triangle>> frames(middleFramesNum, std::vector<Triangle>());

        int objectNums = viewer.keyframes[0].size();
        for (int i = 0; i < objectNums; i++) {
            auto &cur = viewer.keyframes[0][i];
            std::vector<Eigen::Vector4f> control_points;
            for (auto &kf: viewer.keyframes) {
                control_points.push_back(kf[i].centroid());
            }
            for (int j = 0; j < middleFramesNum; j++) {
                Triangle middle = cur;
                auto oldCenter = middle.centroid();
                auto newCenter = bezier_re(control_points, float(j)/float(middleFramesNum));
                auto offset = newCenter - oldCenter;
                for (auto &v: middle.vs) {
                    v.position += offset;
                }
                frames[j].push_back(middle);
            }
        }

        for (auto &f: frames) {
            draw(program, frameBuffer, viewer, f);
        }
    };
}


int main() {
    int width = 500;
    int height = 500;
    // The Framebuffer storing the image rendered by the rasterizer
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(width, height);

    // Basic rasterization program
    Program program;

    // The vertex shader is the identity
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        VertexAttributes ret = va;

        ret.position = {ret.position.x() / float(uniform.width) * 2 - 1, (float(uniform.height)-ret.position.y())/float(uniform.height)*2 -1 , 0, 1};

        ret.position = uniform.view_Matri * ret.position;
        ret.color = va.color;
        return ret;
    };

    // The fragment shader uses a fixed color
    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) -> FragmentAttributes {
        return FragmentAttributes(va.color(0),va.color(1),va.color(2));
    };

    // The blending shader converts colors between 0 and 1 to uint8
    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    UniformAttributes &uniform = viewer.uniform;
    viewer.init("Viewer Example", width, height);

    uniform.width = width;
    uniform.height = height;

    viewer.key_pressed = [&](char key, bool key_press, int modifier, int repeat) {
        if (key != 'n') {
            viewer.redraw = [&](SDLViewer &viewer) {
                draw(program, frameBuffer, viewer, viewer.triangles);
            };
        }
       
        switch (key) {
            case 'i': {
                viewer.reset();
                viewer.mode = 'i';
                viewer.mouse_move = [&](int x, int y, int xrel, int yrel) {
                    auto nx = float(x);
                    auto ny = float(y);
                    inver_view(viewer.uniform, nx, ny);

                    while (viewer.insertionBuffer.size() > viewer.clickCount && !viewer.insertionBuffer.empty()) {
                        viewer.insertionBuffer.pop_back();
                    }
                    viewer.insertionBuffer.emplace_back(nx, ny, 0, 1);
                    viewer.redraw_next = true;
                };
                viewer.mouse_pressed = [&](int x, int y, bool key_press, int button, int clicks) {
                    auto nx = float(x);
                    auto ny = float(y);
                    inver_view(viewer.uniform, nx, ny);

                    if (key_press) {
                        viewer.clickCount += 1;
                        viewer.insertionBuffer.emplace_back(nx, ny, 0, 1);
                        if (viewer.clickCount == 3) {
                            viewer.triangles.emplace_back(viewer.insertionBuffer[0], viewer.insertionBuffer[1],
                                                        viewer.insertionBuffer[2]);
                            viewer.insertionBuffer.clear();
                            viewer.clickCount = 0;
                            viewer.redraw_next = true;
                        }
                    }

                };
                break;
            }
            case 'o': {
                viewer.reset();
                viewer.mode = 'o';
                viewer.mouse_pressed =[&](int x, int y, bool key_press, int button, int clicks) {
                    auto nx = float(x);
                    auto ny = float(y);
                    inver_view(viewer.uniform, nx, ny);

                    if (key_press) {
                        Eigen::Vector4f pt(nx, ny, 0, 1);
                        int index = find_triangle(viewer.triangles, pt);
                        if (index >= 0) {
                            viewer.click_index = index;
                            auto &triangle = viewer.triangles[viewer.click_index];
                            triangle.highlight = true;
                            viewer.drag = true;
                        } else {
                            viewer.drag = false;
                            viewer.clear_lighr();
                        }
                    } else {
                        viewer.drag = false;
                        viewer.clear_lighr();
                    }
                    viewer.redraw_next = true;
                };

                viewer.mouse_move = [&](int x, int y, int xrel, int yrel) {
                    auto nxrel = float(xrel);
                    auto nyrel = float(yrel);

                    Eigen::Vector4f vec(nxrel, nyrel, 0, 1);
                    vec.x() = vec.x() * viewer.uniform.inver_Matri(0,0);
                    vec.y() = vec.y() * viewer.uniform.inver_Matri(1,1);
                    nxrel = vec.x();
                    nyrel = vec.y();
                   
                    if (viewer.drag && viewer.click_index >= 0) {
                        auto &triangle = viewer.triangles[viewer.click_index];
                        triangle.shift(float(nxrel), float(nyrel));
                    }
                    viewer.redraw_next = true;
                };
                break;
            }
            case 'p': {
                viewer.reset();
                viewer.mode = 'p';
                viewer.mouse_pressed = [&](int x, int y, bool key_press, int button, int clicks) {
                    if (key_press) {
                        auto nx = float(x);
                        auto ny = float(y);
                        inver_view(viewer.uniform, nx, ny);

                        Eigen::Vector4f pt(nx, ny, 0, 1);
                        int index = find_triangle(viewer.triangles, pt);
                        viewer.click_index = index;
                        if (index >= 0) {
                            viewer.triangles.erase(viewer.triangles.begin() + index);
                        }
                        viewer.redraw_next = true;
                    }
                };
                break;
            }
            case 'h':
            case 'j':
            case 'k':
            case 'l':
                if (viewer.mode == 'o' && key_press) {
                    float degree;
                    float factor;
                    if(key == 'h'){
                        degree=10.f;
                    }
                    else if(key == 'j'){
                        degree=-10.f;
                    }
                    else degree=0.f;

                    if(key == 'k'){
                        factor=1.25f;
                    }
                    else if(key == 'l'){
                        factor=0.75f;
                    }
                    else factor=1.f;

                    if (viewer.click_index >= 0) {
                        auto &t = viewer.triangles[viewer.click_index];
                        t.rotate(degree);
                    }
                    if (viewer.click_index >= 0) {
                        auto &t = viewer.triangles[viewer.click_index];
                        t.scale(factor);
                    }
                }
                viewer.redraw_next = true;
                break;
            case 'c':
                viewer.reset();
                viewer.mode = 'c';
                viewer.mouse_pressed = [&](int x, int y, bool key_press, int button, int clicks) {
                    if (key_press) {
                        auto nx = float(x);
                        auto ny = float(y);
                        inver_view(viewer.uniform, nx, ny);

                        if (!viewer.triangles.empty()) {
                            Eigen::Vector4f pt(nx, ny, 0, 1);
                            
                            int tri_index = 0;
                            int ver_index = 0;
                            float minDist = 10000000.f;
                            for (int i = 0 ; i < viewer.triangles.size();i++) {
                                auto &triangle = viewer.triangles[i];
                                for (int j = 0 ; j < 3; j++) {
                                    auto &v = triangle.vs[j];
                                    float d =(v.position.x()- pt.x())*(v.position.x()- pt.x()) + (v.position.y()-pt.y())*(v.position.y()-pt.y());
                                    if (d < minDist) {
                                        minDist = d;
                                        tri_index = i;
                                        ver_index = j;
                                    }
                                }
                            }
                            auto pair =std::make_pair(tri_index, ver_index);
                            
                            viewer.tri_color = pair.first;
                            viewer.ver_color = pair.second;
                            viewer.redraw_next = true;
                        }
                    }
                };
                break;
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8':
            case '9':
                if (viewer.mode == 'c') {
                    if (viewer.tri_color >= 0 && viewer.tri_color < viewer.triangles.size() && viewer.ver_color >= 0 && viewer.ver_color < 3) {
                        int color = key - '0';
                        float r = 0.3f * float(color/3);
                        float g = 0.3f * float(color%3);
                        float b = 1-(r+g)/2;
                        auto &v = viewer.triangles[viewer.tri_color].vs[viewer.ver_color];
                        v.color.x() = r;
                        v.color.y() = g;
                        v.color.z() = b;
                    }
                    viewer.redraw_next = true;
                }
                break;
            case '=':
                if (key_press) {
                    zoom_matrix(uniform, 1);
                    viewer.redraw_next = true;
                }
                break;
            case '-':
                if (key_press) {
                    zoom_matrix(uniform, -1);
                    viewer.redraw_next = true;
                }
                break;
            case 'w':
                if (key_press) {
                    translate_matrix(uniform, 0, 0.2f);
                    viewer.redraw_next = true;
                }
                break;
            case 'a':
                if (key_press) {
                    translate_matrix(uniform, -0.2f, 0);
                    viewer.redraw_next = true;
                }
                break;
            case 's':
                if (key_press) {
                    translate_matrix(uniform, 0, -0.2f);
                    viewer.redraw_next = true;
                }
                break;
            case 'd':
                if (key_press) {
                    translate_matrix(uniform, 0.2f, 0);
                    viewer.redraw_next = true;
                }
                break;
            case 'm':
                if (key_press) {
                    if (!viewer.triangles.empty()) {
                        viewer.keyframes.push_back(viewer.triangles);
                    }
                }
                break;
            case 'n':
                if (key_press) {
                    if (!viewer.keyframes.empty()) {
                        viewer.redraw = linear_ani(program, frameBuffer);
                        viewer.animate = true;
                    }
                    viewer.redraw_next = true;
                }
                break;
            case 'b':
                if (key_press) {
                    if (!viewer.keyframes.empty()) {
                        viewer.redraw = bezier_ani(program, frameBuffer);
                        viewer.animate = true;
                    }
                    viewer.redraw_next = true;
                }
                break;
            case 'x':
                if (key_press) {
                    viewer.keyframes.clear();
                }
            default:
                break;
        }
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        draw(program, frameBuffer, viewer, viewer.triangles);
    };

    viewer.launch();

    return 0;
}
