//
// Created by pecoffet on 09/07/18.
//

#ifndef ROBOROBO3_DEBUGCOLLOBJ_H
#define ROBOROBO3_DEBUGCOLLOBJ_H


#include <ext/World/RoundObject.h>

class DebugCollObj : public RoundObject {
public:
    explicit DebugCollObj(int __id);
    void isPushed(int id, std::tuple<double, double> speed) override;
    void step() override;

    bool _pushed;
};


#endif //ROBOROBO3_DEBUGCOLLOBJ_H
