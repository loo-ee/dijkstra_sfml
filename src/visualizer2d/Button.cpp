#include "Button.h"

Button::Button(const sf::Vector2f& position, const sf::Vector2f& size) {
    m_buttonShape.setSize(size);
    m_buttonShape.setPosition(position);
    
    // Default slate colors
    m_normalColor = sf::Color(30, 41, 59);
    m_hoverColor = sf::Color(51, 65, 85);
    m_activeColor = sf::Color(14, 165, 233);
    m_textNormalColor = sf::Color(226, 232, 240);
    m_textHoverColor = sf::Color::White;

    m_buttonShape.setFillColor(m_normalColor);
    m_buttonShape.setOutlineThickness(1.f);
    m_buttonShape.setOutlineColor(sf::Color(71, 85, 105));
}

void Button::setPosition(const sf::Vector2f& position) {
    m_buttonShape.setPosition(position);
    centerText();
}

void Button::setSize(const sf::Vector2f& size) {
    m_buttonShape.setSize(size);
    centerText();
}

void Button::setButtonText(const sf::Font& font, const std::string& text, unsigned int fontSize) {
    m_text.setFont(font);
    m_text.setString(text);
    m_text.setCharacterSize(fontSize);
    m_text.setFillColor(m_textNormalColor);
    centerText();
}

void Button::centerText() {
    if (m_text.getFont() == nullptr) return;
    sf::FloatRect textBounds = m_text.getLocalBounds();
    sf::FloatRect buttonBounds = m_buttonShape.getGlobalBounds();
    m_text.setOrigin(std::floor(textBounds.left + textBounds.width / 2.f), std::floor(textBounds.top + textBounds.height / 2.f));
    m_text.setPosition(std::floor(buttonBounds.left + buttonBounds.width / 2.f), std::floor(buttonBounds.top + buttonBounds.height / 2.f));
}

void Button::setColors(sf::Color normal, sf::Color hover, sf::Color textNormal, sf::Color textHover) {
    m_normalColor = normal;
    m_hoverColor = hover;
    m_textNormalColor = textNormal;
    m_textHoverColor = textHover;
    m_buttonShape.setFillColor(m_normalColor);
    m_text.setFillColor(m_textNormalColor);
}

void Button::setOutline(sf::Color color, float thickness) {
    m_buttonShape.setOutlineColor(color);
    m_buttonShape.setOutlineThickness(thickness);
}

void Button::setCallback(std::function<void()> onClick) {
    m_onClick = onClick;
}

void Button::setActive(bool active) {
    m_isActive = active;
}

void Button::setDisabled(bool disabled) {
    m_isDisabled = disabled;
}

void Button::update(const sf::Vector2f& mousePos) {
    if (m_isDisabled) {
        m_buttonShape.setFillColor(sf::Color(15, 23, 42));
        m_text.setFillColor(sf::Color(100, 116, 139));
        return;
    }

    m_isHovered = m_buttonShape.getGlobalBounds().contains(mousePos);

    if (m_isActive) {
        m_buttonShape.setFillColor(m_activeColor);
        m_text.setFillColor(sf::Color::White);
    } else if (m_isHovered) {
        m_buttonShape.setFillColor(m_hoverColor);
        m_text.setFillColor(m_textHoverColor);
    } else {
        m_buttonShape.setFillColor(m_normalColor);
        m_text.setFillColor(m_textNormalColor);
    }
}

void Button::handleEvent(const sf::Event& event, const sf::RenderWindow& window) {
    if (m_isDisabled) return;

    if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
        sf::Vector2f mousePos = window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x, event.mouseButton.y));
        if (m_buttonShape.getGlobalBounds().contains(mousePos)) {
            if (m_onClick) {
                m_onClick();
            }
        }
    }
}

void Button::draw(sf::RenderTarget& target, sf::RenderStates states) const {
    target.draw(m_buttonShape, states);
    target.draw(m_text, states);
}
