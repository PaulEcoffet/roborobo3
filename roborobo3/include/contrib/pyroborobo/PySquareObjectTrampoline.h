//
// Created by Paul Ecoffet on 25/09/2020.
//

#ifndef ROBOROBO3_PYSQUAREOBJECTTRAMPOLINE_H
#define ROBOROBO3_PYSQUAREOBJECTTRAMPOLINE_H

#include <core/World/SquareObject.h>
#include <pybind11/pybind11.h>


template<class BaseSquare = SquareObject>
class PySquareObjectTrampoline : public BaseSquare
{
public:
    using BaseSquare::BaseSquare;

    void step() override
    {
        PYBIND11_OVERLOAD(void, BaseSquare, step,);
    }

    bool canRegister() override
    {
        PYBIND11_OVERLOAD_NAME(bool, BaseSquare, "can_register", canRegister,);
    }

    void registerObject() override
    {
        PYBIND11_OVERLOAD_NAME(void, BaseSquare, "register_object", registerObject,);
    }

    void unregisterObject() override
    {
        PYBIND11_OVERLOAD_NAME(void, BaseSquare, "unregister_object", unregisterObject,);
    }

    void isTouched(int _idAgent) override
    {
        PYBIND11_OVERLOAD_NAME(void, BaseSquare, "is_touched", isTouched, _idAgent);
    }

    void isWalked(int _idAgent) override
    {
        PYBIND11_OVERLOAD_NAME(void, BaseSquare, "is_walked", isWalked, _idAgent);
    }

    void isPushed(int _id, std::tuple<double, double> _speed) override
    {
        PYBIND11_OVERLOAD_NAME(void, BaseSquare, "is_pushed", isPushed, _id, _speed);
    }

    void show(SDL_Surface *surface) override
    {
        PYBIND11_OVERLOAD(void, BaseSquare, show, surface);
    }

    void hide() override
    {
        PYBIND11_OVERLOAD(void, BaseSquare, hide,);
    }

    std::string inspect(std::string prefix) override
    {
        PYBIND11_OVERLOAD(std::string, BaseSquare, inspect, prefix);
    }

    virtual void relocate()
    {
        PYBIND11_OVERLOAD(void, BaseSquare, relocate,);
    }

    virtual bool relocate(int x, int y)
    {
        PYBIND11_OVERLOAD(bool, BaseSquare, relocate, x, y);
    }


};


#endif //ROBOROBO3_PYSQUAREOBJECTTRAMPOLINE_H
