if (msg.payload === "1.00") { //sjekker inngangsverdi
    msg.payload = "ÅPEN"      //endrer til "ÅPEN", hvis msg.payload === "1.00"
    return msg;               //returnerer ny msg.payload
}


if(msg.payload === "0.00") {  //sjekker inngangsverdi
    msg.payload = "STENGT"    //endrer til "STENGT", hvis msg.payload === "0.00"
    return msg;               //returnerer ny msg.payload
}
