#pragma once

#include <SFML/Graphics.hpp>
#include <functional>

class Button : public sf::Drawable, public sf::Transformable {
public:
    Button(const sf::Vector2f& position, const sf::Vector2f& size);

    void handleEvent(const sf::Event& event, const sf::RenderWindow& window);

    void setButtonText(sf::Font& font, const std::string& text, int fontSize);
    void setButtonColor(sf::Color color);
    void setButtonTextColor(sf::Color color);
    void setCallback(std::function<void()> onClick);

private:
    sf::RectangleShape m_buttonShape;
    sf::Vector2f m_position;
    sf::Vector2f m_size;
    sf::Text m_text;
    sf::Font m_font;
    std::function<void()> m_onClick;

    void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
        target.draw(this->m_buttonShape, states);
        target.draw(this->m_text, states);
    }
};

