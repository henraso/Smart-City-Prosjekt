Node RED flow.json er hele kildekoden til alt i Node RED, og kan importeres for å se og ha tilgang til alt tilknyttet dashbordet og noder i Node RED

I tillegg er koden fra enkelte av de egendefinerte funksjonene lagt til her.  

"Endre farge på knapp.js" er en funksjon som setter knappen som aktiverer trafikklyset til den fargen lyset faktisk er, enten rødt eller grønt.

"Endre msg.payload garasje.js" er en enkel funksjon som endrer verdien som er lagret i msg.payload for å brukes videre, her da for å vise en mer forståelig verdi i dashbordet.  

"Egenlaget_funksjon_Node_RED_for_salg_av_strom.js" er den mest avanserte noden. Her blir en verdi mottatt kontinuerlig fra solcelleanlegget, som fyller på et fiktivt batteri. Verdien der blir lagret i en global verdi. I tillegg så er det en knapp som selger strømmen, da trekkes en verdi fra batteriet, og lagres i en ny variabel som sendes videre til Zumo32U4. Alt dette måtte læres på egenhånd, da funksjonsnodene er bygget i programmeringsspråket Javascript. 
