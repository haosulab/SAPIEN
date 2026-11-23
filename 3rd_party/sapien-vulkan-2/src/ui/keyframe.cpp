/*
 * Copyright 2025 Hillbot Inc.
 * Copyright 2020-2024 UCSD SU Lab
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "svulkan2/ui/keyframe.h"
#include <algorithm>
#include <imgui.h>
#include <imgui_internal.h>
#include <set>
#include <string>
#include <stdexcept>

namespace svulkan2 {
namespace ui {

static auto KeyframeCmp = [](std::shared_ptr<Keyframe> a, std::shared_ptr<Keyframe> b) {
  return a->frame() < b->frame();
};

class KeyframeEditorImpl : public KeyframeEditor {
public:
  KeyframeEditorImpl(float contentScale);
  void build() override;

  void addKeyframe(std::shared_ptr<Keyframe> frame) override;
  void removeKeyframe(std::shared_ptr<Keyframe> frame) override;

  void addDuration(std::shared_ptr<Duration> duration) override;
  void removeDuration(std::shared_ptr<Duration> duration) override;

  void setState(std::vector<std::shared_ptr<Keyframe>>,
                std::vector<std::shared_ptr<Duration>>) override;
  std::vector<std::shared_ptr<Keyframe>> getKeyframes() override;
  std::vector<std::shared_ptr<Duration>> getDurations() override;

private:
  void buildControlPanel();
  void buildCrossBackground(ImVec2 X);
  void buildEditorBackground(ImVec2 canvasSize, ImVec2 C);
  void buildVerticalGrid(ImVec2 canvasSize, ImVec2 X, ImVec2 C);
  void buildEditor(ImVec2 canvasSize, ImVec2 X, ImVec2 A, ImVec2 B);
  void buildListerBackground(ImVec2 canvasSize, ImVec2 A);
  void buildLister(ImVec2 canvasSize, ImVec2 X, ImVec2 A, ImVec2 C);
  void buildTimelineBackground(ImVec2 canvasSize, ImVec2 B);
  void buildTimeline(ImVec2 canvasSize, ImVec2 X, ImVec2 B);

  void buildKeyframeIndicators(ImVec2 canvasSize, ImVec2 X, ImVec2 B, ImVec2 C);

  void buildCurrentFrameIndicator(ImVec2 canvasSize, ImVec2 X, ImVec2 B, ImVec2 C);
  void buildHorizScrollbar(ImVec2 canvasSize, ImVec2 X, ImVec2 C);
  void buildVertScrollbar(ImVec2 canvasSize, ImVec2 X, ImVec2 C);
  void buildListerWidthHandle(ImVec2 canvasSize, ImVec2 X, ImVec2 A, ImVec2 B);
  void buildMiddleButtonPanning(ImVec2 canvasSize);

  int mCurrentFrame{};
  int mTotalFrames{};
  std::shared_ptr<Keyframe> mCurrentKeyframe{};

  // // Control panel
  bool mAddingDuration{};
  std::vector<std::shared_ptr<Keyframe>> mKeyframesForNewDuration;

  std::shared_ptr<Keyframe> mDraggedFrame{};
  int mDraggedTime{-1};

  std::vector<std::shared_ptr<Keyframe>> mKeyframes;
  std::vector<std::shared_ptr<Duration>> mDurations;
  std::array<int, 2> mTotalFrameRange{32, 2048};
  int mStride{1};

  // Visual
  float mContentScale{1.f};
  float mPan[2]; // Deviation of {timeline, lister} in pixels
  float mInitialPan[2];
  float mZoom[2]; // Distance between each {frame, duration} in pixels
  float mHorizZoomRange[2];
  float mListerInitialWidth{};

  // Theme
  struct CrossTheme_ {
    float borderWidth{1.0f};

    ImVec4 background{ImColor::HSV(0.0f, 0.0f, 0.114f)};
    ImVec4 border{ImColor::HSV(0.0f, 0.0f, 0.2f)};
  } CrossTheme;

  struct ListerTheme_ {
    float width{100.0f};
    float handleWidth{15.0f};

    ImVec4 background{ImColor::HSV(0.0f, 0.0f, 0.114f)};
    ImVec4 text{ImColor::HSV(0.0f, 0.0f, 0.7f)};
  } ListerTheme;

  struct TimelineTheme_ {
    float height{40.0f};
    float keyframeIndicatorSize{12.0f};
    float currentFrameIndicatorSize{22.0f};

    ImVec4 background{ImColor::HSV(0.0f, 0.0f, 0.114f)};
    ImVec4 text{ImColor::HSV(0.0f, 0.0f, 0.7f)};
    ImVec4 dark{ImColor::HSV(0.0f, 0.0f, 0.075f)};
    ImVec4 currentFrame{ImColor::HSV(0.6f, 0.6f, 0.95f)};
    ImVec4 keyframe{ImColor::HSV(0.12f, 0.8f, 0.95f)};
    ImVec4 selectedKeyframe{ImColor::HSV(0.75f, 0.25f, 0.95f)};
  } TimelineTheme;

  struct EditorTheme_ {
    float scrollbarPadding{20.0f};
    float scrollbarSize{10.0f};
    float durationHeight{15.0f};

    ImVec4 background{ImColor::HSV(0.0f, 0.0f, 0.188f)};
    ImVec4 dark{ImColor::HSV(0.0f, 0.0f, 0.075f)};
    ImVec4 mid{ImColor::HSV(0.0f, 0.0f, 0.15f)};
    ImVec4 scrollbar{ImColor::HSV(0.0f, 0.0f, 0.33f)};
    ImVec4 duration{ImColor::HSV(0.12f, 0.8f, 0.9f)};
  } EditorTheme;
};

void KeyframeEditorImpl::addKeyframe(std::shared_ptr<Keyframe> frame) {
  if (std::find(mKeyframes.begin(), mKeyframes.end(), frame) != mKeyframes.end()) {
    throw std::runtime_error("frame is added twice");
  }
  mKeyframes.push_back(frame);
  std::sort(mKeyframes.begin(), mKeyframes.end(), KeyframeCmp);
}

void KeyframeEditorImpl::removeKeyframe(std::shared_ptr<Keyframe> frame) {
  std::erase_if(mDurations, [=](std::shared_ptr<Duration> &d) {
    return d->keyframe0() == frame || d->keyframe1() == frame;
  });
  std::erase(mKeyframes, frame);
  // std::sort(mKeyframes.begin(), mKeyframes.end(), KeyframeCmp);
}

void KeyframeEditorImpl::addDuration(std::shared_ptr<Duration> duration) {
  auto f0 = duration->keyframe0();
  auto f1 = duration->keyframe1();
  if (std::find(mKeyframes.begin(), mKeyframes.end(), f0) == mKeyframes.end() ||
      std::find(mKeyframes.begin(), mKeyframes.end(), f1) == mKeyframes.end()) {
    throw std::runtime_error("duration contains removed frames");
  }
  if (std::find(mDurations.begin(), mDurations.end(), duration) != mDurations.end()) {
    throw std::runtime_error("duration is added twice");
  }
  mDurations.push_back(duration);
}

void KeyframeEditorImpl::removeDuration(std::shared_ptr<Duration> duration) {
  if (std::erase(mDurations, duration) == 0) {
    throw std::runtime_error("duration is already removed");
  }
}

void KeyframeEditorImpl::setState(std::vector<std::shared_ptr<Keyframe>> frames,
                                  std::vector<std::shared_ptr<Duration>> durations) {
  // check not null
  for (auto frame : frames) {
    if (!frame) {
      throw std::runtime_error("some frame is null");
    }
  }
  for (auto duration : durations) {
    if (!duration) {
      throw std::runtime_error("some duration is null");
    }
  }

  // check no duplicated frames
  std::sort(frames.begin(), frames.end(), KeyframeCmp);
  for (size_t i = 1; i < frames.size(); ++i) {
    if (frames[i - 1]->frame() == frames[i]->frame()) {
      throw std::runtime_error("some frames have the same time stamp");
    }
  }

  // check durations are linked to keyframes
  std::set<std::shared_ptr<Keyframe>> frameSet(frames.begin(), frames.end());
  for (auto duration : durations) {
    auto f0 = duration->keyframe0();
    auto f1 = duration->keyframe1();
    if (!f0 || !f1) {
      throw std::runtime_error("some duration contains invalid keyframe");
    }
    if (!frameSet.contains(f0) || !frameSet.contains(f1)) {
      throw std::runtime_error(
          "some duration contains keyframes that are not in the keyframe list");
    }
  }

  mKeyframes.clear();
  mDurations.clear();
  mKeyframes = std::vector(frames.begin(), frames.end());
  mDurations = std::vector(durations.begin(), durations.end());

  std::sort(mKeyframes.begin(), mKeyframes.end(), KeyframeCmp);

  // TODO: check validity, no duplicate frames, durations are linked to frames
}
std::vector<std::shared_ptr<Keyframe>> KeyframeEditorImpl::getKeyframes() {
  return std::vector(mKeyframes.begin(), mKeyframes.end());
}
std::vector<std::shared_ptr<Duration>> KeyframeEditorImpl::getDurations() { return mDurations; }

KeyframeEditorImpl::KeyframeEditorImpl(float contentScale) {
  // Visual
  if (contentScale < 0.1f) {
    mContentScale = 1.0f;
  } else {
    mContentScale = contentScale;
  }
  mPan[0] = 0.0f;
  mPan[1] = 0.0f;
  mZoom[0] = 25.0f * mContentScale;
  mHorizZoomRange[1] = 100.0f * mContentScale;

  CrossTheme = {};
  ListerTheme = {};
  TimelineTheme = {};
  EditorTheme = {};

  // Theme
  CrossTheme.borderWidth *= mContentScale;
  ListerTheme.width *= mContentScale;
  ListerTheme.handleWidth *= mContentScale;
  TimelineTheme.height *= mContentScale;
  TimelineTheme.keyframeIndicatorSize *= mContentScale;
  TimelineTheme.currentFrameIndicatorSize *= mContentScale;
  EditorTheme.scrollbarPadding *= mContentScale;
  EditorTheme.scrollbarSize *= mContentScale;
  EditorTheme.durationHeight *= mContentScale;
}

void KeyframeEditorImpl::buildControlPanel() {
  if (mAddingDuration) {
    if (ImGui::Button("Exit")) {
      mKeyframesForNewDuration.clear();
      mAddingDuration = false;
    }
    if (mKeyframesForNewDuration.size() == 0) {
      ImGui::SameLine();
      ImGui::Text("Select first key frame for the new duration:");
    } else if (mKeyframesForNewDuration.size() == 1) {
      ImGui::SameLine();
      ImGui::Text("Select second key frame for the new duration:");
    }
    return;
  }
  // Normal mode
  // Current frame setter
  ImGui::PushItemWidth(50.0f * mContentScale);
  if (ImGui::DragInt("Current Frame", &mCurrentFrame, 1.0f, 0, mTotalFrames - 1, "%d",
                     ImGuiSliderFlags_AlwaysClamp)) {
    if (mCurrentFrameSetter) {
      mCurrentFrameSetter(mCurrentFrame);
    }
  }
  ImGui::PopItemWidth();

  // Total frame setter
  int minTotalFrame = mTotalFrameRange[0];
  if (mKeyframes.size() != 0) {
    auto lastFrame = *std::prev(mKeyframes.end());
    minTotalFrame = std::max(minTotalFrame, lastFrame->frame() + 1);
  }

  ImGui::SameLine();
  ImGui::PushItemWidth(50.0f * mContentScale);
  if (ImGui::DragInt("Total Frames", &mTotalFrames, 1.0f, minTotalFrame, mTotalFrameRange[1], "%d",
                     ImGuiSliderFlags_AlwaysClamp)) {
    if (mTotalFramesSetter) {
      mTotalFramesSetter(mTotalFrames);
    }
  }
  ImGui::PopItemWidth();

  if (mCurrentFrame >= mTotalFrames || mCurrentFrame < 0) {
    if (mCurrentFrameSetter) {
      mCurrentFrameSetter(ImClamp(mCurrentFrame, 0, mTotalFrames - 1));
    }
  }

  for (uint32_t i = 0; i < mChildren.size(); ++i) {
    ImGui::SameLine();
    mChildren[i]->build();
  }
}

void KeyframeEditorImpl::buildCrossBackground(ImVec2 X) {
  auto drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(X, X + ImVec2{ListerTheme.width, TimelineTheme.height},
                          ImColor(CrossTheme.background));

  drawList->AddLine(X + ImVec2{ListerTheme.width - 1.0f, 0.0f},
                    X + ImVec2{ListerTheme.width - 1.0f, TimelineTheme.height},
                    ImColor(CrossTheme.border), CrossTheme.borderWidth);

  drawList->AddLine(X + ImVec2{0.0f, TimelineTheme.height - 1.0f},
                    X + ImVec2{ListerTheme.width, TimelineTheme.height - 1.0f},
                    ImColor(CrossTheme.border), CrossTheme.borderWidth);
}

void KeyframeEditorImpl::buildEditorBackground(ImVec2 canvasSize, ImVec2 C) {
  auto drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(C, C + canvasSize - ImVec2{ListerTheme.width, TimelineTheme.height},
                          ImColor(EditorTheme.background));

  // Mouse event
  ImGui::SetCursorPos(ImVec2{C.x, C.y} - ImGui::GetWindowPos());
  ImGui::InvisibleButton(
      "##EditorBackground",
      ImVec2{canvasSize.x - ListerTheme.width, canvasSize.y - TimelineTheme.height},
      ImGuiButtonFlags_AllowItemOverlap);
  ImGui::SetItemAllowOverlap();

  if (ImGui::IsItemHovered()) {
    ImGui::SetItemUsingMouseWheel();

    // Horizontal scrolling
    float minHorizMPan = -(mTotalFrames * mZoom[0] - (canvasSize.x - ListerTheme.width));
    minHorizMPan = std::min(minHorizMPan, 0.0f);
    mPan[0] = ImClamp(mPan[0] + ImGui::GetIO().MouseWheelH * mZoom[0], minHorizMPan, 0.0f);

    // Vertical scrolling
    float minVertMPan = -(static_cast<int>(mDurations.size()) * mZoom[1] + 5.0f * mContentScale -
                          (canvasSize.y - TimelineTheme.height));
    minVertMPan = std::min(minVertMPan, 0.0f);
    mPan[1] = ImClamp(mPan[1] + ImGui::GetIO().MouseWheel * mZoom[1], minVertMPan, 0.0f);
  }
}

void KeyframeEditorImpl::buildVerticalGrid(ImVec2 canvasSize, ImVec2 X, ImVec2 C) {
  auto drawList = ImGui::GetWindowDrawList();
  float interspace = mZoom[0] * mStride;                  // Distance between each base frame
  int frameStart = ceil(-mPan[0] / interspace) * mStride; // Only draw in visible area
  frameStart = ImClamp(frameStart, 0, mTotalFrames - 1);
  float xMidStart = C.x + frameStart / mStride * interspace + mPan[0] - interspace / 2;
  float yMin = C.y;
  float yMax = C.y + canvasSize.y - TimelineTheme.height;

  if (mStride > 1 && xMidStart > C.x) { // First mid line
    drawList->AddLine(ImVec2(xMidStart, yMin), ImVec2(xMidStart, yMax), ImColor(EditorTheme.mid),
                      1.0f * mContentScale);
  }

  for (int frame = frameStart; frame <= mTotalFrames - 1; frame += mStride) {
    int i = frame / mStride; // ith line

    float x = C.x + i * interspace + mPan[0];
    if (x > X.x + canvasSize.x) { // Only draw in visible area
      break;
    }

    drawList->AddLine(ImVec2(x, yMin), ImVec2(x, yMax), ImColor(EditorTheme.dark),
                      1.0f * mContentScale);

    float xMid = x + interspace / 2;
    if (mStride > 1 && xMid < X.x + canvasSize.x) { // Draw when mStride > 1 and visible
      drawList->AddLine(ImVec2(xMid, yMin), ImVec2(xMid, yMax), ImColor(EditorTheme.mid),
                        1.0f * mContentScale);
    }
  }
}

void KeyframeEditorImpl::buildEditor(ImVec2 canvasSize, ImVec2 X, ImVec2 A, ImVec2 B) {
  auto drawList = ImGui::GetWindowDrawList();

  for (int i = 0, id = 0; i < static_cast<int>(mDurations.size()); i++, id++) {
    float y = std::round(A.y + 5.0f * mContentScale + mZoom[1] * i +
                         mPan[1]); // Avoid sub-pixel rendering
    if (y < A.y) {                 // Start drawing from top of lister
      continue;
    }
    ImVec2 textSize = ImGui::CalcTextSize("test");
    float itemHeight = std::max(textSize.y, EditorTheme.durationHeight);
    if (y + itemHeight > X.y + canvasSize.y) { // End drawing when exceeds bottom
      break;
    }

    auto duration = mDurations[i];

    auto f0 = duration->keyframe0();
    auto f1 = duration->keyframe1();

    int frameA = f0 == mDraggedFrame ? mDraggedTime : f0->frame();
    int frameB = f1 == mDraggedFrame ? mDraggedTime : f1->frame();

    if (frameA == frameB) {
      continue;
    }

    int frameStart = (frameA < frameB) ? frameA : frameB;
    int frameEnd = (frameA < frameB) ? frameB : frameA;
    float xStart = B.x + frameStart * mZoom[0] + mPan[0];
    float xEnd = B.x + frameEnd * mZoom[0] + mPan[0];
    if (xEnd <= B.x || xStart >= X.x + canvasSize.x) {
      continue;
    }

    float btnXStart = (xStart < B.x) ? B.x : xStart;
    float btnXEnd = (xEnd > X.x + canvasSize.x) ? X.x + canvasSize.x : xEnd;

    ImVec4 color = EditorTheme.duration;

    // Mouse event
    ImGui::PushID(id);
    ImGui::SetCursorPos(ImVec2{btnXStart, y} - ImGui::GetWindowPos());
    ImGui::InvisibleButton("##Duration", ImVec2{btnXEnd - btnXStart, EditorTheme.durationHeight},
                           ImGuiButtonFlags_AllowItemOverlap);
    ImGui::SetItemAllowOverlap();
    ImGui::PopID();

    if (ImGui::IsItemHovered()) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    }

    if (ImGui::IsItemClicked() && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
      if (mDoubleClickDurationCallback) {
        mDoubleClickDurationCallback(duration);
      }
    }

    if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) {
      removeDuration(duration);
      break; // stop drawing the rest
    }

    ImVec2 rectMin(xStart, y);
    ImVec2 rectMax(xEnd, y + EditorTheme.durationHeight);
    ImColor contourColor(0, 0, 0, 255);
    float contourThickness = 1.2f * mContentScale;
    drawList->AddRectFilled(rectMin, rectMax, ImColor(color), 2.0f * mContentScale);
    drawList->AddRect(rectMin, rectMax, contourColor, 2.0f * mContentScale, 0, contourThickness);
  }
}

void KeyframeEditorImpl::buildListerBackground(ImVec2 canvasSize, ImVec2 A) {
  auto drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(A, A + ImVec2{ListerTheme.width, canvasSize.y - TimelineTheme.height},
                          ImColor(ListerTheme.background));
}

void KeyframeEditorImpl::buildLister(ImVec2 canvasSize, ImVec2 X, ImVec2 A, ImVec2 C) {
  auto drawList = ImGui::GetWindowDrawList();
  float x = A.x + 10.0f * mContentScale;
  for (int i = 0; i < static_cast<int>(mDurations.size()); i++) {
    float y = std::round(A.y + 5.0f * mContentScale + mZoom[1] * i +
                         mPan[1]); // Avoid sub-pixel rendering
    if (y < A.y) {                 // Start drawing from top of lister
      continue;
    }
    ImVec2 textSize = ImGui::CalcTextSize("test");
    float itemHeight = std::max(textSize.y, EditorTheme.durationHeight);
    if (y + itemHeight > X.y + canvasSize.y) { // End drawing when exceeds bottom
      break;
    }

    auto duration = mDurations[i];
    std::string name = duration->name();
    std::string visibleName;
    for (int i = name.size(); i > 0; i--) { // Find visible substring
      visibleName = name.substr(0, i);
      ImVec2 textSize = ImGui::CalcTextSize(visibleName.c_str());
      if (x + textSize.x <= C.x) {
        break;
      }
    }

    if (visibleName.size() > 0) {
      // Mouse event
      ImGui::PushID(i);
      ImGui::SetCursorPos(ImVec2{x, y} - ImGui::GetWindowPos());
      ImVec2 textSize = ImGui::CalcTextSize(visibleName.c_str());
      ImGui::InvisibleButton("##DurationName", textSize, ImGuiButtonFlags_AllowItemOverlap);
      ImGui::SetItemAllowOverlap();
      ImGui::PopID();

      if (ImGui::IsItemHovered()) {
        ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
      }

      if (ImGui::BeginDragDropSource()) {
        int sourceIndex = i;
        ImGui::SetDragDropPayload("DragDuration", &sourceIndex, sizeof(int));
        ImGui::Text("%s", visibleName.c_str());
        ImGui::EndDragDropSource();
      }

      if (ImGui::BeginDragDropTarget()) {
        if (const ImGuiPayload *payload = ImGui::AcceptDragDropPayload("DragDuration")) {
          IM_ASSERT(payload->DataSize == sizeof(int));
          int sourceIndex = *(const int *)payload->Data;
          int targetIndex = i;
          auto item = mDurations[sourceIndex];
          mDurations.erase(mDurations.begin() + sourceIndex);
          mDurations.insert(mDurations.begin() + targetIndex, item);
          ImGui::EndDragDropTarget();
          break; // DurationsInUsed updated, stop iterating
        } else {
          ImGui::EndDragDropTarget();
        }
      }

      drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize(), ImVec2{x, y},
                        ImColor(ListerTheme.text), visibleName.c_str());
    }
  }
}

void KeyframeEditorImpl::buildTimelineBackground(ImVec2 canvasSize, ImVec2 B) {
  auto drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(B, B + ImVec2{canvasSize.x - ListerTheme.width, TimelineTheme.height},
                          ImColor(TimelineTheme.background));

  // Mouse event
  ImGui::SetCursorPos(ImVec2{B.x, B.y} - ImGui::GetWindowPos());
  ImGui::InvisibleButton("##TimelineBackground",
                         ImVec2{canvasSize.x - ListerTheme.width, TimelineTheme.height},
                         ImGuiButtonFlags_AllowItemOverlap);
  ImGui::SetItemAllowOverlap();

  if (ImGui::IsItemHovered()) {
    ImGui::SetItemUsingMouseWheel();

    // Zooming
    float initialZoom = mZoom[0];
    mZoom[0] =
        ImClamp(mZoom[0] + ImGui::GetIO().MouseWheel, mHorizZoomRange[0], mHorizZoomRange[1]);

    // Change mPan according to mZoom
    if (mZoom[0] != initialZoom) {
      float minHorizPan = -(mTotalFrames * mZoom[0] - (canvasSize.x - ListerTheme.width));
      minHorizPan = std::min(minHorizPan, 0.0f);
      float panTemp = mPan[0] / initialZoom * mZoom[0];
      mPan[0] = ImClamp(panTemp, minHorizPan, 0.0f);
    }
  }
  if (!mAddingDuration && ImGui::IsItemActive()) {
    int frameTemp =
        static_cast<int>(std::round((ImGui::GetIO().MousePos.x - B.x - mPan[0]) / mZoom[0]));
    mCurrentFrame = ImClamp(frameTemp, 0, mTotalFrames - 1);
    if (mCurrentFrameSetter) {
      mCurrentFrameSetter(mCurrentFrame);
    }
  }

  if (!mAddingDuration && ImGui::IsItemClicked() && ImGui::IsItemActive() &&
      (ImGui::IsKeyDown(ImGuiKey_LeftShift) || ImGui::IsKeyDown(ImGuiKey_RightShift))) {
    if (std::find_if(mKeyframes.begin(), mKeyframes.end(), [this](auto &f) {
          return f->frame() == mCurrentFrame;
        }) == mKeyframes.end()) {
      if (mAddKeyframeCallback) {
        mAddKeyframeCallback(mCurrentFrame);
      }
    }
  }
}

void KeyframeEditorImpl::buildTimeline(ImVec2 canvasSize, ImVec2 X, ImVec2 B) {
  auto drawList = ImGui::GetWindowDrawList();
  float interspace = mZoom[0] * mStride;                  // Distance between each base frame
  int frameStart = ceil(-mPan[0] / interspace) * mStride; // Only draw in visible area
  frameStart = ImClamp(frameStart, 0, mTotalFrames - 1);
  float yMin = B.y;
  float yMax = B.y + TimelineTheme.height;

  for (int frame = frameStart; frame <= mTotalFrames - 1; frame += mStride) {
    int i = frame / mStride; // ith line

    float x = B.x + i * interspace + mPan[0];
    if (x > X.x + canvasSize.x) { // Only draw in visible area
      break;
    }

    drawList->AddLine(ImVec2(x, yMin), ImVec2(x, yMax), ImColor(TimelineTheme.dark),
                      1.0f * mContentScale);

    ImVec2 textSize = ImGui::CalcTextSize(std::to_string(frame).c_str()) * 0.85f;
    if (x + 5.0f * mContentScale + textSize.x <= X.x + canvasSize.x) { // Only draw in visible area
      drawList->AddText(ImGui::GetFont(), ImGui::GetFontSize() * 0.85f,
                        ImVec2{x + 5.0f * mContentScale, yMin}, ImColor(TimelineTheme.text),
                        std::to_string(frame).c_str());
    }
  }
}

void KeyframeEditorImpl::buildKeyframeIndicators(ImVec2 canvasSize, ImVec2 X, ImVec2 B, ImVec2 C) {
  auto drawList = ImGui::GetWindowDrawList();
  for (int i = 0, id = 0; i < mKeyframes.size(); ++i, ++id) {
    auto frame = mKeyframes[i];

    ImVec4 color = TimelineTheme.keyframe;
    if (mAddingDuration && mKeyframesForNewDuration.size() &&
        mKeyframesForNewDuration[0] == frame) {
      color = TimelineTheme.selectedKeyframe;
    }

    int frameTime = frame->frame();
    if (mDraggedFrame == frame) {
      frameTime = mDraggedTime;
    }

    float x = B.x + frameTime * mZoom[0] + mPan[0];
    if (x < B.x || x > X.x + canvasSize.x) { // Only draw in visible area
      continue;
    }
    float size = TimelineTheme.keyframeIndicatorSize;
    ImVec2 center = ImVec2{x, C.y - size};

    // Calculate the diamond's corner points
    ImVec2 top(center.x, center.y - size / 2);
    ImVec2 right(center.x + size / 2, center.y);
    ImVec2 bottom(center.x, center.y + size / 2);
    ImVec2 left(center.x - size / 2, center.y);

    // Mouse event
    ImGui::PushID(id);
    ImGui::SetCursorPos(ImVec2{left.x, top.y} - ImGui::GetWindowPos());
    ImGui::InvisibleButton("##Keyframe", ImVec2{size, size}, ImGuiButtonFlags_AllowItemOverlap);
    ImGui::SetItemAllowOverlap();
    ImGui::PopID();

    if (ImGui::IsItemHovered()) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
    }

    if (ImGui::IsItemClicked() && ImGui::IsMouseDoubleClicked(0)) {
      if (mDoubleClickKeyframeCallback) {
        mDoubleClickKeyframeCallback(frame);
      }
    }

    if (ImGui::IsItemClicked() &&
        (ImGui::IsKeyDown(ImGuiKey_LeftCtrl) || ImGui::IsKeyDown(ImGuiKey_RightCtrl)) &&
        !mAddingDuration && mKeyframes.size() >= 2) {

      if (mCurrentKeyframe && frame != mCurrentKeyframe) {
        // if we sit on one frame and click another, add duration immediately
        if (mAddDurationCallback) {
          mAddDurationCallback(mCurrentKeyframe, frame);
        }
      } else {
        // otherwise, choose duration
        mAddingDuration = true;
        mKeyframesForNewDuration.push_back(frame);
        continue;
      }
    }

    if (ImGui::IsItemHovered() && ImGui::IsMouseDown(ImGuiMouseButton_Right)) {
      removeKeyframe(frame);
      i--;
      continue;
    }

    if (mAddingDuration) { // Select first and second key frames for new duration
      if (mKeyframesForNewDuration.size() == 0) {
        if (ImGui::IsItemClicked()) {
          mKeyframesForNewDuration.push_back(frame);
        }
      } else if (mKeyframesForNewDuration.size() == 1) {
        if (ImGui::IsItemClicked()) {
          auto it =
              std::find(mKeyframesForNewDuration.begin(), mKeyframesForNewDuration.end(), frame);
          if (it != mKeyframesForNewDuration.end()) { // Deselect key frame
            mKeyframesForNewDuration.erase(it);
          } else {
            auto frame0 = mKeyframesForNewDuration.at(0);
            mKeyframesForNewDuration.clear();
            mAddingDuration = false;
            if (mAddDurationCallback) {
              mAddDurationCallback(frame0, frame);
            }
          }
        }
      }
    } else { // Drag key frame

      if (ImGui::IsItemActive()) {
        color = ImVec4{color.x * 1.2f, color.y * 1.2f, color.z * 1.2f, color.w};
        mDraggedFrame = frame;
        mDraggedTime =
            static_cast<int>(std::round((ImGui::GetIO().MousePos.x - B.x - mPan[0]) / mZoom[0]));
        mDraggedTime = ImClamp(mDraggedTime, 0, mTotalFrames - 1);
        mCurrentFrame = mDraggedTime;
        if (mCurrentFrameSetter) {
          mCurrentFrameSetter(mCurrentFrame);
        }
      }

      if (ImGui::IsItemDeactivated()) {

        auto it = std::find_if(mKeyframes.begin(), mKeyframes.end(), [this](auto &f) {
          return mDraggedTime == f->frame() && mDraggedFrame != f;
        });
        if (it != mKeyframes.end()) { // Has duplicate
          removeKeyframe(*it);
        }

        if (mMoveKeyframeCallback) {
          mMoveKeyframeCallback(frame, mDraggedTime);
          std::sort(mKeyframes.begin(), mKeyframes.end(), KeyframeCmp);
        }

        mDraggedFrame.reset();
        mDraggedTime = -1;
      }
    }

    // Color and thickness for contour
    ImColor contourColor(0, 0, 0, 255);
    float contourThickness = 1.0f * mContentScale;

    drawList->AddQuadFilled(top, right, bottom, left, ImColor(color));
    drawList->AddQuad(top, right, bottom, left, contourColor, contourThickness);
  }
}

void KeyframeEditorImpl::buildCurrentFrameIndicator(ImVec2 canvasSize, ImVec2 X, ImVec2 B,
                                                    ImVec2 C) {
  ImVec4 color = TimelineTheme.currentFrame;
  if (mCurrentKeyframe || mCurrentFrame == mDraggedTime) {
    color = TimelineTheme.keyframe;
  }
  // auto it = std::find_if(mKeyframes.begin(), mKeyframes.end(),
  //                        [&](auto &f) { return f->frame() == mCurrentFrame; });
  // if (it != mKeyframes.end() || mCurrentFrame == mDraggedTime) {
  //   color = TimelineTheme.keyframe;
  // }

  auto drawList = ImGui::GetWindowDrawList();
  float x = B.x + mCurrentFrame * mZoom[0] + mPan[0];
  if (x < B.x || x > X.x + canvasSize.x) { // Only draw in visible area
    return;
  }

  float innerSize = TimelineTheme.keyframeIndicatorSize;
  float outerSize = TimelineTheme.currentFrameIndicatorSize;
  ImVec2 center = ImVec2{x, C.y - innerSize};

  // Calculate the inner diamond's corner points
  ImVec2 innerTop(center.x, center.y - innerSize / 2);
  ImVec2 innerRight(center.x + innerSize / 2, center.y);
  ImVec2 innerBottom(center.x, center.y + innerSize / 2);
  ImVec2 innerLeft(center.x - innerSize / 2, center.y);

  // Calculate the outer diamond's corner points
  ImVec2 outerTop(center.x, center.y - outerSize / 2);
  ImVec2 outerRight(center.x + outerSize / 2, center.y);
  ImVec2 outerBottom(center.x, center.y + outerSize / 2);
  ImVec2 outerLeft(center.x - outerSize / 2, center.y);

  // Color and thickness for contour
  ImColor contourColor(0, 0, 0, 255);
  float contourThickness = 1.0f * mContentScale;

  drawList->AddQuadFilled(outerTop, outerRight, outerBottom, outerLeft, ImColor(color));
  drawList->AddQuad(outerTop, outerRight, outerBottom, outerLeft, contourColor, contourThickness);
  drawList->AddQuad(innerTop, innerRight, innerBottom, innerLeft, contourColor, contourThickness);

  if (canvasSize.y - TimelineTheme.height > 0) { // Editor area exists
    float yMin = C.y;
    float yMax = C.y + canvasSize.y - TimelineTheme.height;
    float lineWidth = outerSize / 10;
    drawList->AddLine(ImVec2{x, yMin}, ImVec2{x, yMax}, ImColor(color), lineWidth);
    drawList->AddLine(ImVec2{x - lineWidth / 2, yMin}, ImVec2{x - lineWidth / 2, yMax},
                      contourColor, 1.0f);
    drawList->AddLine(ImVec2{x + lineWidth / 2, yMin}, ImVec2{x + lineWidth / 2, yMax},
                      contourColor, 1.0f);
  }
}

void KeyframeEditorImpl::buildHorizScrollbar(ImVec2 canvasSize, ImVec2 X, ImVec2 C) {
  auto drawList = ImGui::GetWindowDrawList();
  float barPadding = EditorTheme.scrollbarPadding;
  float areaWidth = canvasSize.x - ListerTheme.width - 2 * barPadding;
  float height = EditorTheme.scrollbarSize;
  float heightMin = X.y + canvasSize.y - height - 4.0f * mContentScale;

  if (areaWidth > 0 && heightMin > C.y) {
    float offsetRatio = -mPan[0] / (mZoom[0] * mTotalFrames);
    float offsetWidth = areaWidth * offsetRatio;
    float barWidthRatio = (canvasSize.x - ListerTheme.width) / (mTotalFrames * mZoom[0]);
    float rawBarWidth = areaWidth * barWidthRatio;
    float barWidth = (rawBarWidth < barPadding - 1.0f * mContentScale)
                         ? barPadding - 1.0f * mContentScale
                         : rawBarWidth; // Ensure grabbable

    ImVec2 barMin = ImVec2{C.x + barPadding + offsetWidth, heightMin};
    ImVec2 barMax = ImVec2{C.x + barPadding + offsetWidth + barWidth, heightMin + height};
    ImVec4 color = EditorTheme.scrollbar;

    ImGui::SetCursorPos(barMin - ImGui::GetWindowPos());
    ImGui::InvisibleButton("##HorizScrollbar", ImVec2{barWidth, height},
                           ImGuiButtonFlags_AllowItemOverlap);
    ImGui::SetItemAllowOverlap();

    if (ImGui::IsItemActivated()) {
      mInitialPan[0] = mPan[0];
    }

    if (ImGui::IsItemActive()) {
      color = ImVec4{color.x * 1.2f, color.y * 1.2f, color.z * 1.2f, color.w};

      float minPan = -(mTotalFrames * mZoom[0] - (canvasSize.x - ListerTheme.width));
      minPan = std::min(minPan, 0.0f);
      float deltaPan = (ImGui::GetMouseDragDelta().x / (areaWidth - rawBarWidth)) * minPan;
      float panTemp = mInitialPan[0] + deltaPan;
      mPan[0] = ImClamp(panTemp, minPan, 0.0f);
    }

    drawList->AddRectFilled(barMin, barMax, ImColor(color), 8.0f * mContentScale);
  }
}

void KeyframeEditorImpl::buildVertScrollbar(ImVec2 canvasSize, ImVec2 X, ImVec2 C) {
  auto drawList = ImGui::GetWindowDrawList();
  float barPadding = EditorTheme.scrollbarPadding;
  float areaHeight = canvasSize.y - TimelineTheme.height - 2 * barPadding;
  float width = EditorTheme.scrollbarSize;
  float widthMin = X.x + canvasSize.x - width - 8.0f * mContentScale;

  if (areaHeight > 0 && widthMin > C.x) {
    float offsetRatio = -mPan[1] / (mZoom[1] * static_cast<int>(mDurations.size()));
    float offsetHeight = areaHeight * offsetRatio;
    float barHeightRatio =
        (canvasSize.y - TimelineTheme.height) /
        (static_cast<int>(mDurations.size()) * mZoom[1] + 10.0f * mContentScale);
    float rawBarHeight = areaHeight * barHeightRatio;
    float barHeight = (rawBarHeight < barPadding - 1.0f * mContentScale)
                          ? barPadding - 1.0f * mContentScale
                          : rawBarHeight; // Ensure grabbable

    ImVec2 barMin = ImVec2{widthMin, C.y + barPadding + offsetHeight};
    ImVec2 barMax = ImVec2{widthMin + width, C.y + barPadding + offsetHeight + barHeight};
    ImVec4 color = EditorTheme.scrollbar;

    ImGui::SetCursorPos(barMin - ImGui::GetWindowPos());
    ImGui::InvisibleButton("##VertScrollbar", ImVec2{width, barHeight},
                           ImGuiButtonFlags_AllowItemOverlap);
    ImGui::SetItemAllowOverlap();

    if (ImGui::IsItemActivated()) {
      mInitialPan[1] = mPan[1];
    }

    if (ImGui::IsItemActive()) {
      color = ImVec4{color.x * 1.2f, color.y * 1.2f, color.z * 1.2f, color.w};

      float minPan = -(static_cast<int>(mDurations.size()) * mZoom[1] + 5.0f * mContentScale -
                       (canvasSize.y - TimelineTheme.height));
      minPan = std::min(minPan, 0.0f);
      float deltaPan = (ImGui::GetMouseDragDelta().y / (areaHeight - rawBarHeight)) * minPan;
      float panTemp = mInitialPan[1] + deltaPan;
      mPan[1] = ImClamp(panTemp, minPan, 0.0f);
    }

    drawList->AddRectFilled(barMin, barMax, ImColor(color), 8.0f * mContentScale);
  }
}

void KeyframeEditorImpl::buildListerWidthHandle(ImVec2 canvasSize, ImVec2 X, ImVec2 A, ImVec2 B) {
  float buttonLeftX = std::max(B.x - ListerTheme.handleWidth / 2, X.x);
  float buttonRightX = std::min(B.x + ListerTheme.handleWidth / 2, X.x + canvasSize.x);
  float adaptedHandleWidth = buttonRightX - buttonLeftX;
  if (adaptedHandleWidth > 0) {
    ImGui::SetCursorPos(ImVec2{buttonLeftX, A.y} - ImGui::GetWindowPos());
    ImGui::InvisibleButton("##ListerWidthHandle",
                           ImVec2{adaptedHandleWidth, canvasSize.y - TimelineTheme.height},
                           ImGuiButtonFlags_AllowItemOverlap);
    ImGui::SetItemAllowOverlap();

    if (ImGui::IsItemHovered()) {
      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
    }

    if (ImGui::IsItemActivated()) {
      mListerInitialWidth = ListerTheme.width;
    }

    if (ImGui::IsItemActive()) {
      float deltaWidth = ImGui::GetMouseDragDelta().x;
      ListerTheme.width = ImClamp(mListerInitialWidth + deltaWidth, 0.0f, canvasSize.x);
    }
  }
}

void KeyframeEditorImpl::buildMiddleButtonPanning(ImVec2 canvasSize) {
  // Horizontal
  float minHorizPan = -(mTotalFrames * mZoom[0] - (canvasSize.x - ListerTheme.width));
  minHorizPan = std::min(minHorizPan, 0.0f);
  float deltaHorizPan = ImGui::GetIO().MouseDelta.x;
  float horizPanTemp = mPan[0] + deltaHorizPan;
  mPan[0] = ImClamp(horizPanTemp, minHorizPan, 0.0f);

  // Vertical
  float minVertPan = -(static_cast<int>(mDurations.size()) * mZoom[1] + 5.0f * mContentScale -
                       (canvasSize.y - TimelineTheme.height));
  minVertPan = std::min(minVertPan, 0.0f);
  float deltaVertPan = ImGui::GetIO().MouseDelta.y;
  float vertPanTemp = mPan[1] + deltaVertPan;
  mPan[1] = ImClamp(vertPanTemp, minVertPan, 0.0f);
}

void KeyframeEditorImpl::build() {
  // update current time frame
  if (mCurrentFrameGetter) {
    mCurrentFrame = mCurrentFrameGetter();
    mTotalFrames = mTotalFramesGetter();
  }

  // find keyframe under current time frame
  mCurrentKeyframe.reset();
  auto it = std::find_if(mKeyframes.begin(), mKeyframes.end(),
                         [this](auto &f) { return f->frame() == mCurrentFrame; });
  if (it != mKeyframes.end()) {
    mCurrentKeyframe = *it;
  }

  buildControlPanel();

  const ImVec2 canvasPos = ImVec2{ImGui::GetWindowPos().x, ImGui::GetCursorScreenPos().y};
  const ImVec2 canvasSize = ImVec2{ImGui::GetWindowSize().x, ImGui::GetContentRegionAvail().y};
  if (canvasSize.x <= 0 || canvasSize.y <= 0) {
    return;
  }

  // init style
  {
    // Clamp lister width
    ListerTheme.width = ImClamp(ListerTheme.width, 0.0f, canvasSize.x);

    // Update minimum horizontal mZoom range
    float minTimelineLength =
        std::max(canvasSize.x - ListerTheme.width - 0.01f,
                 1024.0f * mContentScale); // 0.01f is to make sure no numerical error will
                                           // cause mHorizZoomRange[0] * mTotalFrames >
                                           // canvasSize.x - ListerTheme.width
    mHorizZoomRange[0] = minTimelineLength / mTotalFrames;

    // Clamp mZoom
    float initialZoom = mZoom[0];
    mZoom[0] = ImClamp(mZoom[0], mHorizZoomRange[0], mHorizZoomRange[1]);

    // Change mPan according to mZoom
    float minHorizPan = -(mTotalFrames * mZoom[0] - (canvasSize.x - ListerTheme.width));
    minHorizPan = std::min(minHorizPan, 0.0f);
    float panTemp = mPan[0] / initialZoom * mZoom[0];
    mPan[0] = ImClamp(panTemp, minHorizPan, 0.0f);

    // Update mStride
    mStride = 1;
    float minHorizZoom = mHorizZoomRange[1] / 2; // For mStride == 1
    while (mZoom[0] <= minHorizZoom) {
      mStride *= 2;
      minHorizZoom /= 2;
    }

    // Compute vertical mZoom
    ImVec2 textSize = ImGui::CalcTextSize("test");
    mZoom[1] = std::max(textSize.y, EditorTheme.durationHeight) * 1.5;

    // Clampe vertical mPan
    float minVertPan = -(static_cast<int>(mDurations.size()) * mZoom[1] + 5.0f * mContentScale -
                         (canvasSize.y - TimelineTheme.height));
    minVertPan = std::min(minVertPan, 0.0f);
    mPan[1] = ImClamp(mPan[1], minVertPan, 0.0f);
  }

  /**
   * Division
   *
   *          _________________________________________________
   *         |       |                                         |
   *   Cross |   X   |                  B                      | Timeline
   *         |_______|_________________________________________|
   *         |       |                                         |
   *         |       |                                         |
   *         |       |                                         |
   *  Lister |   A   |                  C                      | Editor
   *         |       |                                         |
   *         |       |                                         |
   *         |_______|_________________________________________|
   *
   */
  ImVec2 X = canvasPos;
  ImVec2 A = canvasPos + ImVec2{0.0f, TimelineTheme.height};
  ImVec2 B = canvasPos + ImVec2{ListerTheme.width, 0.0f};
  ImVec2 C = canvasPos + ImVec2{ListerTheme.width, TimelineTheme.height};

  ImGui::BeginGroup();

  // Cross elements
  if (ListerTheme.width > 0) {
    buildCrossBackground(X);
  }

  // Editor elements
  if (canvasSize.y - TimelineTheme.height > 0 && canvasSize.x - ListerTheme.width > 0) {
    buildEditorBackground(canvasSize, C);
    buildVerticalGrid(canvasSize, X, C);
    buildEditor(canvasSize, X, A, B);
  }

  // Lister elements
  if (ListerTheme.width > 0 && canvasSize.y - TimelineTheme.height > 0) {
    buildListerBackground(canvasSize, A);
    buildLister(canvasSize, X, A, C);
  }

  // Timeline elements
  if (canvasSize.x - ListerTheme.width > 0) {
    buildTimelineBackground(canvasSize, B);
    buildTimeline(canvasSize, X, B);

    if (!mAddingDuration) {
      buildCurrentFrameIndicator(canvasSize, X, B, C);
    }

    buildKeyframeIndicators(canvasSize, X, B, C);
  }

  // Horizontal scrollbar
  if (mTotalFrames * mZoom[0] > canvasSize.x - ListerTheme.width) {
    buildHorizScrollbar(canvasSize, X, C);
  }

  // Vertical scrollbar
  if (static_cast<int>(mDurations.size()) * mZoom[1] + 10.0f * mContentScale >
      canvasSize.y - TimelineTheme.height) {
    buildVertScrollbar(canvasSize, X, C);
  }

  // Lister width handle
  buildListerWidthHandle(canvasSize, X, A, B);

  // Middle button panning
  if (ImGui::IsWindowHovered() && ImGui::GetIO().MouseDown[2]) {
    buildMiddleButtonPanning(canvasSize);
  }

  ImGui::EndGroup();
}

std::shared_ptr<KeyframeEditor> KeyframeEditor::Create(float contentScale) {
  return std::make_shared<KeyframeEditorImpl>(contentScale);
}

} // namespace ui
} // namespace svulkan2