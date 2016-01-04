
//Rebem l'error de posicio amb un node

    //do something


int posx = 10;
int posy = 10;


//Passem l'error a velocitat. Depèn dels pixels que està, anem més ràpid o més lents (X)

if (0 > posx < 319){          //girem motor esquerra

    int velx = 3/16 * posx;
    velx = velx * -1; // Canviem el número de signe
    // Passem al PID

}

if (321 > posx < 640){          //girem motor dreta

    posx = posx - 320;

    int velx = 3/16 * posx;

    // Passem al PID

}

if (posx == 320){ // No hi ha error. Aturem motor

    velx = 0;

}

if (posx < 0){

    velx = -60;
}


if (posx > 640){

    velx = 60;
}


//Passem l'error a velocitat. Depèn dels pixels que està, anem més ràpid o més lents (Y)

if (0 > posy < 239){          //girem motor esquerra

    int vely = 1/4 * posy;
    vely = vely * -1; // Canviem el número de signe
    // Passem al PID

}

if (241 > posy < 480){          //girem motor dreta

    posy = posy - 240;

    int vely = 1/4 * posy;

    // Passem al PID

}

if (posy == 320){ // No hi ha error. Aturem motor

    vely = 0;

}

if (posy < 0){

    vely = -60;
}


if (posy > 640){

    vely = 60;
}

