#pragma once

#ifdef __EMSCRIPTEN__
#include "sfml_web_shim.hpp"
#else
#include <SFML/Graphics.hpp>
#endif
#include <functional>
#include <string>

class Button : public sf::Drawable {
public:
    Button(const sf::Vector2f& position, const sf::Vector2f& size);

    void setPosition(const sf::Vector2f& position);
    void setSize(const sf::Vector2f& size);

    void handleEvent(const sf::Event& event, const sf::RenderWindow& window);
    void update(const sf::Vector2f& mousePos);

    void setButtonText(const sf::Font& font, const std::string& text, unsigned int fontSize = 16);
    void setColors(sf::Color normal, sf::Color hover, sf::Color textNormal, sf::Color textHover = sf::Color::White);
    void setOutline(sf::Color color, float thickness = 1.f);
    void setCallback(std::function<void()> onClick);
    void setActive(bool active);
    void setDisabled(bool disabled);

    bool isActive() const { return m_isActive; }
    bool isDisabled() const { return m_isDisabled; }
    sf::FloatRect getBounds() const { return m_buttonShape.getGlobalBounds(); }

private:
    sf::RectangleShape m_buttonShape;
    sf::Text m_text;
    std::function<void()> m_onClick;

    sf::Color m_normalColor;
    sf::Color m_hoverColor;
    sf::Color m_activeColor;
    sf::Color m_textNormalColor;
    sf::Color m_textHoverColor;

    bool m_isHovered = false;
    bool m_isActive = false;
    bool m_isDisabled = false;

    void centerText();
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override;
};
