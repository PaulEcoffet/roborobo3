//
// Created by Paul Ecoffet on 17/09/2020.
//

#ifndef ROBOROBO3_PYPHYSICALOBJECTTRAMPOLINE_H
#define ROBOROBO3_PYPHYSICALOBJECTTRAMPOLINE_H


template<class ObjectBase = PhysicalObject>
class PyPhysicalObjectTrampoline : public PhysicalObject
{
public:
    using ObjectBase::ObjectBase;

    void step() override
    {
        PYBIND11_OVERLOAD_PURE(void, ObjectBase, step,);
    }

    bool canRegister() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(bool, ObjectBase, "can_register", canRegister,);
    }

    void registerObject() override
    {
        PYBIND11_OVERLOAD_NAME(void, ObjectBase, "register_object", registerObject,);
    }

    void unregisterObject() override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ObjectBase, "unregister_object", unregisterObject,);
    }

    void isTouched(int _idAgent) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ObjectBase, "is_touched", isTouched, _idAgent);
    }

    void isWalked(int _idAgent) override
    {
        PYBIND11_OVERLOAD_PURE_NAME(void, ObjectBase, "is_walked", isWalked, _idAgent);
    }

    void isPushed(int _id, std::tuple<double, double> _speed) override
    {

    }

    virtual void show(SDL_Surface *surface = gScreen) = 0;

    virtual void hide() = 0;

    virtual std::string inspect(std::string prefix = "");

    virtual void relocate();

    virtual bool relocate(int x, int y);


};


#endif //ROBOROBO3_PYPHYSICALOBJECTTRAMPOLINE_H
