#pragma once
// Debug glyph sheet (--glyph-sheet): every glyph at S/M/L, sample HUD strings, a primitives demo
// and an outline-vs-no-outline comparison over a bright and a dark background. Draws into an
// open Renderer2D batch (between begin() and end()).

#include "hud/Renderer2D.h"

namespace fsim {

void drawGlyphSheet(Renderer2D& r);

}  // namespace fsim
