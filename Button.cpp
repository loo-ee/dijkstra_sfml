#include "include/Button.h"

Button::Button(const sf::Vector2f& position, const sf::Vector2f& size) : m_position(position), m_size(size) {
    this->m_buttonShape.setSize(size);
    this->m_buttonShape.setPosition(position);
    this->m_text.setPosition(position + sf::Vector2f(10.f, 10.f));
}

void Button::setButtonText(sf::Font& font, const std::string& text, int fontSize) {
    this->m_font = font;
    this->m_text.setFont(this->m_font);
    this->m_text.setString(text);
    this->m_text.setCharacterSize(fontSize);
}

void Button::setButtonColor(sf::Color color) {
    this->m_buttonShape.setFillColor(color);
}

void Button::setButtonTextColor(sf::Color color) {
    this->m_text.setFillColor(color);
}

void Button::setCallback(std::function<void()> onClick) {
    this->m_onClick = onClick;
}

void Button::handleEvent(const sf::Event& event, const sf::RenderWindow& window) {
    if (event.type != sf::Event::MouseButtonPressed) return;

    sf::Vector2f mousePosition = window.mapPixelToCoords(
        sf::Vector2i(event.mouseButton.x,
            event.mouseButton.y)
    );

    if (this->m_buttonShape.getGlobalBounds().contains(mousePosition)) {
        this->m_onClick();
    }
}

