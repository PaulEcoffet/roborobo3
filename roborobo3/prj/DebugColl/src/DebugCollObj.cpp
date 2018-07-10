//
// Created by pecoffet on 09/07/18.
//

#include "DebugColl/include/DebugCollObj.h"

DebugCollObj::DebugCollObj(int __id) :
        RoundObject(__id),
        _pushed(false)
{
}

void DebugCollObj::isPushed(int id, std::tuple<double, double> speed) {
    _pushed = true;
}

void DebugCollObj::step() {
    _pushed = false;
    RoundObject::step();
}
