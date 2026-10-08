#pragma once

#include <raylib.h>

namespace lf::sim {

// SSoT Design Tokens & Color Palette for LF-Bot Simulator & HUD
struct Theme {
  // Cockpit Background & Surfaces
  static constexpr Color kBg = {11, 15, 23, 255};            // #0B0F17 Cockpit dark
  static constexpr Color kSurface = {17, 24, 39, 255};       // #111827 Flat panel
  static constexpr Color kSurfaceLight = {24, 33, 53, 255};  // #182135 Sub-element
  static constexpr Color kCanvasBg = {13, 17, 23, 255};      // #0D1117 Canvas background

  // Borders & Dividers
  static constexpr Color kBorder = {30, 41, 59, 255};          // #1E293B 1px hairline
  static constexpr Color kBorderLight = {45, 60, 85, 255};     // #2D3C55 Subtle divider
  static constexpr Color kBorderActive = {56, 189, 248, 255};  // #38BDF8 Focus cyan
  static constexpr Color kArenaBorder = {71, 85, 105, 255};    // Arena boundary border

  // Typography
  static constexpr Color kTextMain = {248, 250, 252, 255};   // #F8FAFC High-contrast white
  static constexpr Color kTextMuted = {148, 163, 184, 255};  // #94A3B8 Secondary
  static constexpr Color kTextDim = {100, 116, 139, 255};    // #64748B Tertiary

  // Status & Telemetry Accents
  static constexpr Color kAccent = {56, 189, 248, 255};   // #38BDF8 Cyan
  static constexpr Color kSuccess = {16, 185, 129, 255};  // #10B981 Emerald
  static constexpr Color kWarn = {245, 158, 11, 255};     // #F59E0B Amber
  static constexpr Color kAlert = {239, 68, 68, 255};     // #EF4444 Red

  // Arena & World Canvas
  static constexpr Color kArenaBg = {248, 249, 250, 255};    // White track paper
  static constexpr Color kArenaGrid = {225, 230, 238, 255};  // 200mm grid lines

  // Typography Hierarchy Tokens (px)
  static constexpr float kFontTitle = 26.0f;
  static constexpr float kFontMetric = 22.0f;
  static constexpr float kFontLabel = 15.0f;
  static constexpr float kFontButton = 14.0f;
  static constexpr float kFontSmall = 12.0f;

  // Layout Dimensions (Base unscaled dp)
  static constexpr float kTopPillHeight = 46.0f;
  static constexpr float kRibbonHeight = 52.0f;
  static constexpr float kButtonPaddingX = 14.0f;
  static constexpr float kCardPaddingX = 14.0f;
  static constexpr float kCardGap = 10.0f;
};

// Backwards compatibility alias
using HudTheme = Theme;

}  // namespace lf::sim
