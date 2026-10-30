//
// Created by ye on 2026/10/30.
//

#include "ent_main.hpp"

#include "debug.hpp"

void ent_main()
{
        xTaskCreate(debugTask, "DeBug", 512, NULL, 5, NULL);


}
