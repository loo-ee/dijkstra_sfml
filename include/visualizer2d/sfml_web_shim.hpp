#pragma once

#ifdef __EMSCRIPTEN__

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <map>

namespace sf {

template<typename T>
struct Vector2 {
    T x, y;
    Vector2() : x(0), y(0) {}
    Vector2(T x, T y) : x(x), y(y) {}
    template<typename U>
    explicit Vector2(const Vector2<U>& v) : x(static_cast<T>(v.x)), y(static_cast<T>(v.y)) {}

    Vector2 operator-() const { return Vector2(-x, -y); }
    Vector2 operator+(const Vector2& v) const { return Vector2(x + v.x, y + v.y); }
    Vector2 operator-(const Vector2& v) const { return Vector2(x - v.x, y - v.y); }
    Vector2 operator*(T s) const { return Vector2(x * s, y * s); }
    Vector2 operator/(T s) const { return Vector2(x / s, y / s); }
    Vector2& operator+=(const Vector2& v) { x += v.x; y += v.y; return *this; }
    Vector2& operator-=(const Vector2& v) { x -= v.x; y -= v.y; return *this; }
    Vector2& operator*=(T s) { x *= s; y *= s; return *this; }
    Vector2& operator/=(T s) { x /= s; y /= s; return *this; }
    bool operator==(const Vector2& v) const { return x == v.x && y == v.y; }
    bool operator!=(const Vector2& v) const { return x != v.x || y != v.y; }
};

template<typename T>
inline Vector2<T> operator*(T s, const Vector2<T>& v) {
    return Vector2<T>(v.x * s, v.y * s);
}

typedef Vector2<float> Vector2f;
typedef Vector2<int> Vector2i;
typedef Vector2<unsigned int> Vector2u;

struct Color {
    uint8_t r, g, b, a;
    Color() : r(0), g(0), b(0), a(255) {}
    Color(uint8_t r, uint8_t g, uint8_t b, uint8_t a = 255) : r(r), g(g), b(b), a(a) {}

    static const Color Black;
    static const Color White;
    static const Color Red;
    static const Color Green;
    static const Color Blue;
    static const Color Yellow;
    static const Color Magenta;
    static const Color Cyan;
    static const Color Transparent;

    bool operator==(const Color& c) const { return r == c.r && g == c.g && b == c.b && a == c.a; }
    bool operator!=(const Color& c) const { return !(*this == c); }
};

inline const Color Color::Black(0, 0, 0);
inline const Color Color::White(255, 255, 255);
inline const Color Color::Red(255, 0, 0);
inline const Color Color::Green(0, 255, 0);
inline const Color Color::Blue(0, 0, 255);
inline const Color Color::Yellow(255, 255, 0);
inline const Color Color::Magenta(255, 0, 255);
inline const Color Color::Cyan(0, 255, 255);
inline const Color Color::Transparent(0, 0, 0, 0);

struct FloatRect {
    float left, top, width, height;
    FloatRect() : left(0), top(0), width(0), height(0) {}
    FloatRect(float l, float t, float w, float h) : left(l), top(t), width(w), height(h) {}
    bool contains(float x, float y) const {
        return x >= left && x <= left + width && y >= top && y <= top + height;
    }
    bool contains(const Vector2f& pt) const { return contains(pt.x, pt.y); }
};

class Time {
    int64_t m_microseconds;
public:
    Time() : m_microseconds(0) {}
    explicit Time(int64_t microseconds) : m_microseconds(microseconds) {}
    float asSeconds() const { return static_cast<float>(m_microseconds) / 1000000.f; }
    int32_t asMilliseconds() const { return static_cast<int32_t>(m_microseconds / 1000); }
    int64_t asMicroseconds() const { return m_microseconds; }
    static Time Seconds(float s) { return Time(static_cast<int64_t>(s * 1000000.f)); }
    static Time Milliseconds(int32_t ms) { return Time(static_cast<int64_t>(ms) * 1000); }
};

class Clock {
    uint32_t m_startTime;
public:
    Clock() { restart(); }
    Time getElapsedTime() const {
        uint32_t now = SDL_GetTicks();
        return Time::Milliseconds(now - m_startTime);
    }
    Time restart() {
        Time elapsed = getElapsedTime();
        m_startTime = SDL_GetTicks();
        return elapsed;
    }
};

class View {
    Vector2f m_center{640.f, 360.f};
    Vector2f m_size{1280.f, 720.f};
    FloatRect m_viewport{0.f, 0.f, 1.f, 1.f};
public:
    View() {}
    View(const Vector2f& center, const Vector2f& size) : m_center(center), m_size(size) {}
    void setCenter(float x, float y) { m_center = Vector2f(x, y); }
    void setCenter(const Vector2f& c) { m_center = c; }
    void setSize(float w, float h) { m_size = Vector2f(w, h); }
    void setSize(const Vector2f& s) { m_size = s; }
    void setViewport(const FloatRect& vp) { m_viewport = vp; }
    const Vector2f& getCenter() const { return m_center; }
    const Vector2f& getSize() const { return m_size; }
    const FloatRect& getViewport() const { return m_viewport; }
};

namespace Keyboard {
    enum Key {
        Unknown = -1, Space, Right, Left, R, F11, F, Home, Num0
    };
    inline bool isKeyPressed(Key key) {
        const Uint8* state = SDL_GetKeyboardState(NULL);
        switch (key) {
            case Space: return state[SDL_SCANCODE_SPACE];
            case Right: return state[SDL_SCANCODE_RIGHT];
            case Left: return state[SDL_SCANCODE_LEFT];
            case R: return state[SDL_SCANCODE_R];
            case F11: return state[SDL_SCANCODE_F11];
            case F: return state[SDL_SCANCODE_F];
            case Home: return state[SDL_SCANCODE_HOME];
            case Num0: return state[SDL_SCANCODE_0] || state[SDL_SCANCODE_KP_0];
            default: return false;
        }
    }
}

class RenderWindow;

namespace Mouse {
    enum Button { Left, Right, Middle };
    Vector2i getPosition(const RenderWindow& relativeTo);
}

struct Event {
    enum EventType {
        Closed, Resized, MouseWheelScrolled, MouseButtonPressed, MouseButtonReleased, MouseMoved, KeyPressed
    } type;

    struct SizeEvent { unsigned int width, height; } size;
    struct MouseMoveEvent { int x, y; } mouseMove;
    struct MouseButtonEvent { Mouse::Button button; int x, y; } mouseButton;
    struct MouseWheelScrollEvent { float delta; int x, y; } mouseWheelScroll;
    struct KeyEvent { Keyboard::Key code; } key;
};

class Texture {
public:
    void setSmooth(bool smooth) {}
};

class Font {
    mutable std::map<unsigned int, TTF_Font*> m_fonts;
    mutable Texture m_dummyTexture;
    std::string m_path;
public:
    Font() {}
    ~Font() {
        for (auto& pair : m_fonts) {
            if (pair.second) TTF_CloseFont(pair.second);
        }
    }
    bool loadFromFile(const std::string& filename) {
        m_path = filename;
        if (!TTF_WasInit()) {
            if (TTF_Init() != 0) return false;
        }
        TTF_Font* f = TTF_OpenFont(filename.c_str(), 16);
        if (f) {
            m_fonts[16] = f;
            return true;
        }
        return false;
    }
    TTF_Font* getTTFFont(unsigned int sz) const {
        auto it = m_fonts.find(sz);
        if (it != m_fonts.end()) return it->second;
        if (m_path.empty()) return nullptr;
        TTF_Font* f = TTF_OpenFont(m_path.c_str(), sz);
        if (f) m_fonts[sz] = f;
        return f;
    }
    Texture& getTexture(unsigned int size) const { return m_dummyTexture; }
};

class RenderTarget;
struct RenderStates {};

class Drawable {
public:
    virtual ~Drawable() {}
    virtual void draw(RenderTarget& target, RenderStates states) const = 0;
};

enum PrimitiveType {
    Points, Lines, LineStrip, Triangles, TriangleStrip, TriangleFan, Quads
};

struct Vertex {
    Vector2f position;
    Color color;
    Vector2f texCoords;
    Vertex() {}
    Vertex(const Vector2f& pos, const Color& c) : position(pos), color(c) {}
};

class VertexArray : public Drawable {
    std::vector<Vertex> m_vertices;
    PrimitiveType m_type = Points;
public:
    VertexArray() {}
    VertexArray(PrimitiveType type, size_t vertexCount = 0) : m_type(type), m_vertices(vertexCount) {}
    void append(const Vertex& vertex) { m_vertices.push_back(vertex); }
    void clear() { m_vertices.clear(); }
    size_t getVertexCount() const { return m_vertices.size(); }
    Vertex& operator[](size_t index) { return m_vertices[index]; }
    const Vertex& operator[](size_t index) const { return m_vertices[index]; }
    PrimitiveType getPrimitiveType() const { return m_type; }
    const std::vector<Vertex>& getVertices() const { return m_vertices; }

    void draw(RenderTarget& target, RenderStates states) const override;
};

class RectangleShape : public Drawable {
    Vector2f m_size{0.f, 0.f};
    Vector2f m_position{0.f, 0.f};
    Vector2f m_origin{0.f, 0.f};
    Color m_fillColor{255, 255, 255, 255};
    Color m_outlineColor{0, 0, 0, 255};
    float m_outlineThickness = 0.f;
public:
    RectangleShape() {}
    explicit RectangleShape(const Vector2f& size) : m_size(size) {}
    void setSize(const Vector2f& size) { m_size = size; }
    const Vector2f& getSize() const { return m_size; }
    void setPosition(float x, float y) { m_position = Vector2f(x, y); }
    void setPosition(const Vector2f& pos) { m_position = pos; }
    const Vector2f& getPosition() const { return m_position; }
    void setOrigin(float x, float y) { m_origin = Vector2f(x, y); }
    void setOrigin(const Vector2f& orig) { m_origin = orig; }
    const Vector2f& getOrigin() const { return m_origin; }
    void setFillColor(const Color& color) { m_fillColor = color; }
    const Color& getFillColor() const { return m_fillColor; }
    void setOutlineThickness(float thickness) { m_outlineThickness = thickness; }
    float getOutlineThickness() const { return m_outlineThickness; }
    void setOutlineColor(const Color& color) { m_outlineColor = color; }
    const Color& getOutlineColor() const { return m_outlineColor; }

    FloatRect getGlobalBounds() const {
        return FloatRect(m_position.x - m_origin.x, m_position.y - m_origin.y, m_size.x, m_size.y);
    }
    FloatRect getLocalBounds() const { return FloatRect(0.f, 0.f, m_size.x, m_size.y); }
    void draw(RenderTarget& target, RenderStates states) const override;
};

class CircleShape : public Drawable {
    float m_radius = 0.f;
    Vector2f m_position{0.f, 0.f};
    Vector2f m_origin{0.f, 0.f};
    Color m_fillColor{255, 255, 255, 255};
    Color m_outlineColor{0, 0, 0, 255};
    float m_outlineThickness = 0.f;
public:
    CircleShape(float radius = 0.f) : m_radius(radius) {}
    void setRadius(float radius) { m_radius = radius; }
    float getRadius() const { return m_radius; }
    void setPosition(float x, float y) { m_position = Vector2f(x, y); }
    void setPosition(const Vector2f& pos) { m_position = pos; }
    const Vector2f& getPosition() const { return m_position; }
    void setOrigin(float x, float y) { m_origin = Vector2f(x, y); }
    void setOrigin(const Vector2f& orig) { m_origin = orig; }
    const Vector2f& getOrigin() const { return m_origin; }
    void setFillColor(const Color& color) { m_fillColor = color; }
    const Color& getFillColor() const { return m_fillColor; }
    void setOutlineThickness(float thickness) { m_outlineThickness = thickness; }
    float getOutlineThickness() const { return m_outlineThickness; }
    void setOutlineColor(const Color& color) { m_outlineColor = color; }
    const Color& getOutlineColor() const { return m_outlineColor; }

    FloatRect getGlobalBounds() const {
        return FloatRect(m_position.x - m_origin.x, m_position.y - m_origin.y, m_radius * 2.f, m_radius * 2.f);
    }
    FloatRect getLocalBounds() const { return FloatRect(0.f, 0.f, m_radius * 2.f, m_radius * 2.f); }
    void draw(RenderTarget& target, RenderStates states) const override;
};

class Text : public Drawable {
    std::string m_string;
    const Font* m_font = nullptr;
    unsigned int m_characterSize = 16;
    Color m_fillColor = Color::White;
    Vector2f m_position{0.f, 0.f};
    Vector2f m_origin{0.f, 0.f};
public:
    Text() {}
    Text(const std::string& str, const Font& font, unsigned int characterSize = 30)
        : m_string(str), m_font(&font), m_characterSize(characterSize) {}

    void setFont(const Font& font) { m_font = &font; }
    void setString(const std::string& string) { m_string = string; }
    void setCharacterSize(unsigned int size) { m_characterSize = size; }
    void setFillColor(const Color& color) { m_fillColor = color; }
    void setPosition(float x, float y) { m_position = Vector2f(x, y); }
    void setPosition(const Vector2f& p) { m_position = p; }
    void setOrigin(float x, float y) { m_origin = Vector2f(x, y); }
    void setOrigin(const Vector2f& o) { m_origin = o; }

    const std::string& getString() const { return m_string; }
    const Font* getFont() const { return m_font; }
    FloatRect getLocalBounds() const {
        if (!m_font) return FloatRect(0.f, 0.f, 0.f, 0.f);
        TTF_Font* tf = m_font->getTTFFont(m_characterSize);
        if (!tf) return FloatRect(0.f, 0.f, m_string.length() * (m_characterSize * 0.6f), static_cast<float>(m_characterSize));
        int w = 0, h = 0;
        TTF_SizeText(tf, m_string.c_str(), &w, &h);
        return FloatRect(0.f, 0.f, static_cast<float>(w), static_cast<float>(h));
    }
    FloatRect getGlobalBounds() const {
        FloatRect loc = getLocalBounds();
        return FloatRect(m_position.x - m_origin.x, m_position.y - m_origin.y, loc.width, loc.height);
    }
    void draw(RenderTarget& target, RenderStates states) const override;
};

struct ContextSettings {
    unsigned int antialiasingLevel = 0;
};

struct VideoMode {
    unsigned int width, height;
    VideoMode(unsigned int w, unsigned int h) : width(w), height(h) {}
    static VideoMode getDesktopMode() { return VideoMode(1280, 720); }
};

namespace Style {
    enum { None = 0, Titlebar = 1, Resize = 2, Close = 4, Fullscreen = 8, Default = Titlebar | Resize | Close };
}

class RenderTarget {
protected:
    View m_view;
public:
    virtual ~RenderTarget() {}
    virtual void clear(const Color& color = Color(0, 0, 0)) = 0;
    virtual void draw(const Drawable& drawable, const RenderStates& states = RenderStates()) {
        drawable.draw(*this, states);
    }
    virtual void draw(const VertexArray& vertices, const RenderStates& states = RenderStates()) = 0;
    virtual void setView(const View& view) { m_view = view; }
    virtual const View& getView() const { return m_view; }
    virtual SDL_Renderer* getSDLRenderer() const = 0;
    virtual Vector2u getSize() const = 0;

    Vector2f worldToScreen(const Vector2f& worldPos) const {
        Vector2u windowSize = getSize();
        FloatRect vp = m_view.getViewport();
        float rectX = vp.left * windowSize.x;
        float rectY = vp.top * windowSize.y;
        float rectW = vp.width * windowSize.x;
        float rectH = vp.height * windowSize.y;

        float normX = (worldPos.x - (m_view.getCenter().x - m_view.getSize().x / 2.f)) / m_view.getSize().x;
        float normY = (worldPos.y - (m_view.getCenter().y - m_view.getSize().y / 2.f)) / m_view.getSize().y;

        return Vector2f(rectX + normX * rectW, rectY + normY * rectH);
    }
};

class RenderWindow : public RenderTarget {
    SDL_Window* m_window = nullptr;
    SDL_Renderer* m_renderer = nullptr;
    bool m_isOpen = false;
    Vector2u m_size{1280, 720};
public:
    RenderWindow() {}
    RenderWindow(VideoMode mode, const std::string& title, uint32_t style = Style::Default, const ContextSettings& settings = ContextSettings()) {
        create(mode, title, style, settings);
    }
    ~RenderWindow() {
        if (m_renderer) SDL_DestroyRenderer(m_renderer);
        if (m_window) SDL_DestroyWindow(m_window);
    }

    void create(VideoMode mode, const std::string& title, uint32_t style = Style::Default, const ContextSettings& settings = ContextSettings()) {
        if (m_renderer) { SDL_DestroyRenderer(m_renderer); m_renderer = nullptr; }
        if (m_window) { SDL_DestroyWindow(m_window); m_window = nullptr; }

        if (SDL_WasInit(SDL_INIT_VIDEO) == 0) {
            SDL_Init(SDL_INIT_VIDEO);
        }

        m_size = Vector2u(mode.width, mode.height);
        m_view.setSize(static_cast<float>(mode.width), static_cast<float>(mode.height));
        m_view.setCenter(static_cast<float>(mode.width) / 2.f, static_cast<float>(mode.height) / 2.f);

        m_window = SDL_CreateWindow(title.c_str(), SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, mode.width, mode.height, SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
        m_renderer = SDL_CreateRenderer(m_window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        SDL_SetRenderDrawBlendMode(m_renderer, SDL_BLENDMODE_BLEND);
        m_isOpen = (m_window != nullptr && m_renderer != nullptr);
    }

    bool isOpen() const { return m_isOpen; }
    void close() { m_isOpen = false; }

    bool pollEvent(Event& event) {
        SDL_Event e;
        if (!SDL_PollEvent(&e)) return false;

        switch (e.type) {
            case SDL_QUIT:
                event.type = Event::Closed;
                break;
            case SDL_WINDOWEVENT:
                if (e.window.event == SDL_WINDOWEVENT_RESIZED || e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
                    event.type = Event::Resized;
                    m_size = Vector2u(e.window.data1, e.window.data2);
                    event.size.width = m_size.x;
                    event.size.height = m_size.y;
                } else {
                    return pollEvent(event);
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
            case SDL_MOUSEBUTTONUP:
                event.type = (e.type == SDL_MOUSEBUTTONDOWN) ? Event::MouseButtonPressed : Event::MouseButtonReleased;
                event.mouseButton.x = e.button.x;
                event.mouseButton.y = e.button.y;
                if (e.button.button == SDL_BUTTON_LEFT) event.mouseButton.button = Mouse::Left;
                else if (e.button.button == SDL_BUTTON_RIGHT) event.mouseButton.button = Mouse::Right;
                else if (e.button.button == SDL_BUTTON_MIDDLE) event.mouseButton.button = Mouse::Middle;
                break;
            case SDL_MOUSEMOTION:
                event.type = Event::MouseMoved;
                event.mouseMove.x = e.motion.x;
                event.mouseMove.y = e.motion.y;
                break;
            case SDL_MOUSEWHEEL:
                event.type = Event::MouseWheelScrolled;
                event.mouseWheelScroll.delta = static_cast<float>(e.wheel.y);
                int mx, my;
                SDL_GetMouseState(&mx, &my);
                event.mouseWheelScroll.x = mx;
                event.mouseWheelScroll.y = my;
                break;
            case SDL_KEYDOWN:
                event.type = Event::KeyPressed;
                if (e.key.keysym.sym == SDLK_SPACE) event.key.code = Keyboard::Space;
                else if (e.key.keysym.sym == SDLK_RIGHT) event.key.code = Keyboard::Right;
                else if (e.key.keysym.sym == SDLK_LEFT) event.key.code = Keyboard::Left;
                else if (e.key.keysym.sym == SDLK_r) event.key.code = Keyboard::R;
                else if (e.key.keysym.sym == SDLK_f) event.key.code = Keyboard::F;
                else if (e.key.keysym.sym == SDLK_F11) event.key.code = Keyboard::F11;
                else if (e.key.keysym.sym == SDLK_HOME) event.key.code = Keyboard::Home;
                else if (e.key.keysym.sym == SDLK_0) event.key.code = Keyboard::Num0;
                else event.key.code = Keyboard::Unknown;
                break;
            default:
                return pollEvent(event);
        }
        return true;
    }

    void setFramerateLimit(unsigned int limit) {}

    void clear(const Color& color = Color(0, 0, 0)) override {
        if (!m_renderer) return;
        SDL_SetRenderDrawColor(m_renderer, color.r, color.g, color.b, color.a);
        SDL_RenderClear(m_renderer);
    }

    void draw(const Drawable& drawable, const RenderStates& states = RenderStates()) override {
        drawable.draw(*this, states);
    }

    void draw(const VertexArray& vertices, const RenderStates& states = RenderStates()) override;

    void display() {
        if (m_renderer) SDL_RenderPresent(m_renderer);
    }

    SDL_Renderer* getSDLRenderer() const override { return m_renderer; }
    Vector2u getSize() const override { return m_size; }

    Vector2f mapPixelToCoords(const Vector2i& point, const View& view) const {
        FloatRect vp = view.getViewport();
        float rectX = vp.left * m_size.x;
        float rectY = vp.top * m_size.y;
        float rectW = vp.width * m_size.x;
        float rectH = vp.height * m_size.y;

        float normX = (point.x - rectX) / rectW;
        float normY = (point.y - rectY) / rectH;

        float worldX = view.getCenter().x + (normX - 0.5f) * view.getSize().x;
        float worldY = view.getCenter().y + (normY - 0.5f) * view.getSize().y;

        return Vector2f(worldX, worldY);
    }

    Vector2f mapPixelToCoords(const Vector2i& point) const {
        return mapPixelToCoords(point, m_view);
    }
};

inline Vector2i Mouse::getPosition(const RenderWindow& relativeTo) {
    int x, y;
    SDL_GetMouseState(&x, &y);
    return Vector2i(x, y);
}

// Drawing implementations
inline void RectangleShape::draw(RenderTarget& target, RenderStates states) const {
    SDL_Renderer* ren = target.getSDLRenderer();
    if (!ren) return;

    Vector2f worldTL(m_position.x - m_origin.x, m_position.y - m_origin.y);
    Vector2f worldBR(worldTL.x + m_size.x, worldTL.y + m_size.y);

    Vector2f scrTL = target.worldToScreen(worldTL);
    Vector2f scrBR = target.worldToScreen(worldBR);

    SDL_Rect r{
        static_cast<int>(scrTL.x),
        static_cast<int>(scrTL.y),
        static_cast<int>(scrBR.x - scrTL.x),
        static_cast<int>(scrBR.y - scrTL.y)
    };

    if (m_fillColor.a > 0) {
        SDL_SetRenderDrawColor(ren, m_fillColor.r, m_fillColor.g, m_fillColor.b, m_fillColor.a);
        SDL_RenderFillRect(ren, &r);
    }

    if (m_outlineThickness > 0.f && m_outlineColor.a > 0) {
        SDL_SetRenderDrawColor(ren, m_outlineColor.r, m_outlineColor.g, m_outlineColor.b, m_outlineColor.a);
        SDL_RenderDrawRect(ren, &r);
    }
}

inline void CircleShape::draw(RenderTarget& target, RenderStates states) const {
    SDL_Renderer* ren = target.getSDLRenderer();
    if (!ren) return;

    Vector2f centerWorld(m_position.x - m_origin.x + m_radius, m_position.y - m_origin.y + m_radius);
    Vector2f centerScr = target.worldToScreen(centerWorld);

    Vector2f radWorld(m_position.x - m_origin.x + m_radius * 2.f, m_position.y - m_origin.y);
    Vector2f radScr = target.worldToScreen(radWorld);
    float radiusScr = std::abs(radScr.x - centerScr.x);

    const int segments = 32;
    std::vector<SDL_Vertex> sdlVerts(segments + 1);
    sdlVerts[0].position = { centerScr.x, centerScr.y };
    sdlVerts[0].color = { m_fillColor.r, m_fillColor.g, m_fillColor.b, m_fillColor.a };
    sdlVerts[0].tex_coord = { 0.f, 0.f };

    for (int i = 0; i < segments; i++) {
        float angle = (i * 2.f * M_PI) / segments;
        sdlVerts[i + 1].position = { centerScr.x + radiusScr * std::cos(angle), centerScr.y + radiusScr * std::sin(angle) };
        sdlVerts[i + 1].color = { m_fillColor.r, m_fillColor.g, m_fillColor.b, m_fillColor.a };
        sdlVerts[i + 1].tex_coord = { 0.f, 0.f };
    }

    std::vector<int> indices;
    for (int i = 0; i < segments; i++) {
        indices.push_back(0);
        indices.push_back(i + 1);
        indices.push_back((i + 1) % segments + 1);
    }

    if (m_fillColor.a > 0) {
        SDL_RenderGeometry(ren, NULL, sdlVerts.data(), static_cast<int>(sdlVerts.size()), indices.data(), static_cast<int>(indices.size()));
    }

    if (m_outlineThickness > 0.f && m_outlineColor.a > 0) {
        SDL_SetRenderDrawColor(ren, m_outlineColor.r, m_outlineColor.g, m_outlineColor.b, m_outlineColor.a);
        for (int i = 0; i < segments; i++) {
            float a1 = (i * 2.f * M_PI) / segments;
            float a2 = ((i + 1) * 2.f * M_PI) / segments;
            SDL_RenderDrawLine(ren,
                static_cast<int>(centerScr.x + radiusScr * std::cos(a1)), static_cast<int>(centerScr.y + radiusScr * std::sin(a1)),
                static_cast<int>(centerScr.x + radiusScr * std::cos(a2)), static_cast<int>(centerScr.y + radiusScr * std::sin(a2)));
        }
    }
}

inline void VertexArray::draw(RenderTarget& target, RenderStates states) const {
    SDL_Renderer* ren = target.getSDLRenderer();
    if (!ren || m_vertices.empty()) return;

    if (m_type == Lines) {
        for (size_t i = 0; i + 1 < m_vertices.size(); i += 2) {
            Vector2f s1 = target.worldToScreen(m_vertices[i].position);
            Vector2f s2 = target.worldToScreen(m_vertices[i + 1].position);
            SDL_SetRenderDrawColor(ren, m_vertices[i].color.r, m_vertices[i].color.g, m_vertices[i].color.b, m_vertices[i].color.a);
            SDL_RenderDrawLine(ren, static_cast<int>(s1.x), static_cast<int>(s1.y), static_cast<int>(s2.x), static_cast<int>(s2.y));
        }
    } else if (m_type == Triangles) {
        std::vector<SDL_Vertex> sdlVerts(m_vertices.size());
        for (size_t i = 0; i < m_vertices.size(); i++) {
            Vector2f scr = target.worldToScreen(m_vertices[i].position);
            sdlVerts[i].position = { scr.x, scr.y };
            sdlVerts[i].color = { m_vertices[i].color.r, m_vertices[i].color.g, m_vertices[i].color.b, m_vertices[i].color.a };
            sdlVerts[i].tex_coord = { 0.f, 0.f };
        }
        SDL_RenderGeometry(ren, NULL, sdlVerts.data(), static_cast<int>(sdlVerts.size()), NULL, 0);
    } else if (m_type == Quads) {
        std::vector<SDL_Vertex> sdlVerts;
        std::vector<int> indices;
        for (size_t i = 0; i + 3 < m_vertices.size(); i += 4) {
            int base = static_cast<int>(sdlVerts.size());
            for (int k = 0; k < 4; k++) {
                Vector2f scr = target.worldToScreen(m_vertices[i + k].position);
                SDL_Vertex v;
                v.position = { scr.x, scr.y };
                v.color = { m_vertices[i + k].color.r, m_vertices[i + k].color.g, m_vertices[i + k].color.b, m_vertices[i + k].color.a };
                v.tex_coord = { 0.f, 0.f };
                sdlVerts.push_back(v);
            }
            indices.push_back(base + 0); indices.push_back(base + 1); indices.push_back(base + 2);
            indices.push_back(base + 0); indices.push_back(base + 2); indices.push_back(base + 3);
        }
        SDL_RenderGeometry(ren, NULL, sdlVerts.data(), static_cast<int>(sdlVerts.size()), indices.data(), static_cast<int>(indices.size()));
    }
}

inline void RenderWindow::draw(const VertexArray& vertices, const RenderStates& states) {
    vertices.draw(*this, states);
}

inline void Text::draw(RenderTarget& target, RenderStates states) const {
    SDL_Renderer* ren = target.getSDLRenderer();
    if (!ren || !m_font || m_string.empty()) return;

    TTF_Font* tf = m_font->getTTFFont(m_characterSize);
    if (!tf) return;

    SDL_Color col{ m_fillColor.r, m_fillColor.g, m_fillColor.b, m_fillColor.a };
    SDL_Surface* surf = TTF_RenderUTF8_Blended(tf, m_string.c_str(), col);
    if (!surf) return;

    SDL_Texture* tex = SDL_CreateTextureFromSurface(ren, surf);
    if (!tex) { SDL_FreeSurface(surf); return; }

    Vector2f worldPos(m_position.x - m_origin.x, m_position.y - m_origin.y);
    Vector2f scrPos = target.worldToScreen(worldPos);

    SDL_Rect dst{ static_cast<int>(scrPos.x), static_cast<int>(scrPos.y), surf->w, surf->h };
    SDL_RenderCopy(ren, tex, NULL, &dst);

    SDL_DestroyTexture(tex);
    SDL_FreeSurface(surf);
}

} // namespace sf

#endif
