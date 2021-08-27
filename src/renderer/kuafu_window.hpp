//
// Created by jet on 8/26/21.
//

#pragma once
#include <core/context/global.hpp>

namespace sapien::Renderer {

class KWindow : public kuafu::Window {
public:
  KWindow(int width, int height, const char *title, uint32_t flags, kuafu::Scene *scene) :
      kuafu::Window(width, height, title, flags), pScene(scene) {}

  auto init() -> bool override {
    if (!kuafu::Window::init()) {
      return false;
    }

    SDL_SetRelativeMouseMode(SDL_FALSE);
    return true;
  }

  auto update() -> bool override {
    if (!kuafu::Window::update()) {
      return false;
    }

    pScene->getCamera()->setSize(mWidth, mHeight);

    SDL_Event event;

    while (SDL_PollEvent(&event) != 0) {
      switch (event.type) {
      case SDL_QUIT: {
        return false;
      }

      case SDL_WINDOWEVENT: {
        switch (event.window.event) {
        case SDL_WINDOWEVENT_CLOSE:
          return false;

        case SDL_WINDOWEVENT_RESIZED:
          resize(static_cast<int>(event.window.data1), static_cast<int>(event.window.data2));
          break;

        case SDL_WINDOWEVENT_MINIMIZED:
          resize(0, 0);
          break;
        }
        break;
      }

      case SDL_KEYDOWN: {
        switch (event.key.keysym.sym) {
        case SDLK_w:
          kuafu::global::keys::eW = true;
          break;

        case SDLK_a:
          kuafu::global::keys::eA = true;
          break;

        case SDLK_s:
          kuafu::global::keys::eS = true;
          break;

        case SDLK_d:
          kuafu::global::keys::eD = true;
          break;


        case SDLK_q:
          kuafu::global::keys::eQ = true;
          break;

        case SDLK_e:
          kuafu::global::keys::eE = true;
          break;


        case SDLK_x:
          kuafu::global::keys::eX = true;
          break;

        case SDLK_y:
          kuafu::global::keys::eY = true;
          break;

        case SDLK_z:
          kuafu::global::keys::eZ = true;
          break;

        case SDLK_ESCAPE:
          return false;
        }
        break;
      }

      case SDL_KEYUP: {
        switch (event.key.keysym.sym) {
        case SDLK_w:
          kuafu::global::keys::eW = false;
          break;

        case SDLK_a:
          kuafu::global::keys::eA = false;
          break;

        case SDLK_s:
          kuafu::global::keys::eS = false;
          break;

        case SDLK_d:
          kuafu::global::keys::eD = false;
          break;


        case SDLK_q:
          kuafu::global::keys::eQ = false;
          break;

        case SDLK_e:
          kuafu::global::keys::eE = false;
          break;


        case SDLK_x:
          kuafu::global::keys::eX = false;
          break;

        case SDLK_y:
          kuafu::global::keys::eY = false;
          break;

        case SDLK_z:
          kuafu::global::keys::eZ = false;
          break;
        }
        break;
      }

      case SDL_MOUSEMOTION: {
        if (!mMouseVisible) {
          int x;
          int y;
          SDL_GetRelativeMouseState(&x, &y);
          pScene->getCamera()->processMouse(x, -y);
          break;
        }
      }
      }
    }
    return true;
  }

private:
  kuafu::Scene *pScene;
  bool mMouseVisible = true;
};
}