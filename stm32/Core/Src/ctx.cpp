#include "ctx.hpp"

#include "system.hpp"

AppContext::AppContext() { sys = &System::GetInstance(); }
