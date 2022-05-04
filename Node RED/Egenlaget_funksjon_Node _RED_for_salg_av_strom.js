var powerToSell;    //Kommer fra knappen "Selg strøm", bestemmer hvor mye som skal selges
var moneyReceived;  //Penger man mottar fra salget, og sendes til Zumo
var solarBattery = flow.get("solarBattery") || 0  //definerer en variabel som er tilgjengelig i hele flowen

if(msg.topic === "esp32/solarPanelEnergy") {     //utføres hvis data kommer fra solceller
    var powerReceived = (parseInt(msg.payload)/360); //Kommer fra solcelleanlegget
    solarBattery = solarBattery + powerReceived; //fyller på "batteriet" i Node-RED
    flow.set("solarBattery", solarBattery);      //sender ny verdi som lagres i en variabel tilgjengelig i hele flowen
}

if(msg.payload.Salgssum) {   //hvis strøm blir solgt
    powerToSell = msg.payload.Salgssum;   //setter inngangsverdi til en variabel
    solarBattery = solarBattery - powerToSell;   //Regner ut total
    flow.set("solarbattery", solarBattery);    //oppdaterer variabelen i flowet
    moneyReceived = {payload: powerToSell};    //lagrer en verdi for penger man mottar fra salget
    return[null, moneyReceived];         //returnerer moneyReceived på output 2
}

if(solarBattery >= 50) {  //Setter en maksverdi på batteriet i Node-RED
    flow.set("solarBattery", 50)  //oppdaterer variabelen i flowet
    solarBattery = 50;  //oppdaterer variabelen lokalt
}

moneyReceived = 0; //setter moneyReceived til 0 etter at det er sendt til Zumoen
var displayMoney = {payload: parseInt(parseFloat(solarBattery).toFixed(0))}; //lager en variabel som viser mengden man har tilgjengelig på batteriet

return[displayMoney, null]; //returnerer verdien på batteriet og sender ut på output 1
