#include <Arduino.h>
#include <LcdMenu.h>
#include <MenuScreen.h>
#include <display/LiquidCrystal_I2CAdapter.h>
#include <renderer/CharacterDisplayRenderer.h>

MENU_SCREEN(mainScreen, mainItems,
    ITEM_BASIC("Item 1"),
    ITEM_BASIC("Item 2"),
    ITEM_BASIC("Item 3"),
    ITEM_BASIC("Item 4"));