#! /bin/bash

## Paramètres
#$1 est le fichier que l'on veux compiler

path_pin=/usr/share/arduino/hardware/arduino/variants/standard/ ##Path pour les pins de l'arduino

#### Détection des librairies utilisé par notre fichier source:

cat $1 | grep "#include"| awk -F '[<>"]' '{print $2}' > librairies

#### Edition des liens vers les librairies
cat librairies

## Nombre de libriairies
nbl=$(wc -l < librairies)
echo $nbl
printf "librairie(s) :"

##On créer les liens vers les librairies


#name des librairies

for (( i=1; i<=$nbl; i++ ));do
    libl_var="libl$i"
    libl_value=$(sed -n "${i}p" librairies)
    echo $libl_value
    declare "$libl_var='$libl_value'"
done


if [ -e "lib" ]; then
    sudo rm -r lib
    echo "Supprimé"
fi

#On créer le dossier qui contiendra l'ensemble des chemins à inclure

touch lib



#Path des répertoires des librairies

for (( i=1; i<=$nbl; i++ )); do
    # Display the current library from the librairies file
    echo "Quel chemin pour:"
    name_var="$(sed -n "${i}p" librairies)"
    # Run locate to find the library's path
    locate "$name_var"
    read h
    # We get the directory path of the selected result
    echo $(locate "$name_var" | sed -n "${h}p" | sed 's:/[^/]*$::' ) >> lib
    # Output the path value pour confirmer
    # Dynamically declare the variable and assign the path value
done

#####Vérification des liens vers les différentes librairies
echo "Chemins des bibliothèques :"

## ON rajoutes les bibliothèques telles que cdefs et celle qui sont nécessaires
##echo "/usr/include/x86_64-linux-gnu/bits/" >> lib
##echo "/usr/include/newlib/" >> lib
##echo "/usr/include/x86_64-linux-gnu/" >> lib
echo "/usr/lib/avr/include/util" >> lib
echo "/usr/lib/avr/include/" >> lib
echo "/usr/share/arduino/libraries/SPI/" >> lib
echo "/usr/share/arduino/libraries/Wire/utility/" >> lib
cat lib


LIB_FILE="lib"

# Variable pour stocker les options -L ce qui automatise le tout
LIB_PATHS=""

# Lire chaque ligne du fichier et ajouter l'option -L

#Creation de la variable $LIB_PATHS qui sera utilisée pour avr-gcc pour inclure toutes les librairies

while IFS= read -r line; do #Tant que la ligne n'est pas vide
    LIB_PATHS="$LIB_PATHS -I$line"
done < "$LIB_FILE"

echo $LIB_PATHS

# Creation du fichier archive qui nous sert de fichier temporaire
if [ -d "archive" ];then
    cd archive 
    sudo rm *
    cd
    cd Trying
    rmdir "archive"
fi

mkdir "archive"


cp $1 /home/cesi/Trying/archive

cp lib /home/cesi/Trying/archive

cd archive

##Preprocessing des librairies.c 2 librairies : Arduino.h

### Préprocessing.lib.c en utilisant lib 
echo "début du preprocessing"

for line in $(cat "$LIB_FILE"); do
    for file in "$line/"*.c; do 
        # Vérifiez si des fichiers .c existent dans le répertoire
        if [[ -e "$file" ]]; then
            echo "Compiling: $file librairie"
            # Compilation avec avr-gcc
            avr-gcc -c "$file" -DF_CPU=16000000UL -Os $LIB_PATHS -I"/usr/share/arduino/hardware/arduino/variants/standard/" -mmcu=atmega328p -o"$(basename "$file" .c).o"
        else
            echo "No .c files found in $line/, be better"
        fi
    done
done

### Pour les fichiers .cpp

echo "Preprocessing en c++"
for line in $(cat "$LIB_FILE"); do
    for file in "$line/"*.cpp; do 
        # Vérifiez si des fichiers .c existent dans le répertoire
        if [[ -e "$file" ]]; then
            echo "Compiling: $file librairie"
            # Compilation avec avr-gcc
            avr-g++ -std=gnu++11 -c "$file" -DF_CPU=16000000UL -Os $LIB_PATHS -I"/usr/share/arduino/hardware/arduino/variants/standard/" -mmcu=atmega328p -o"$(basename "$file" .cpp).o"
        else
            echo "No .cpp files found in $line/, be better"
        fi
    done
done

####On bouge Wire.o & Twi.o pour mettre de l'ordre dans la compilation

mv /home/cesi/Trying/archive/Wire.o /home/cesi/Trying/Wire.o

mv /home/cesi/Trying/archive/twi.o /home/cesi/Trying/twi.o 

#### On retire les librairies inutiles qui ne servent à rien

rm /home/cesi/Trying/archive/RTC_DS3231.o
rm /home/cesi/Trying/archive/RTC_Millis.o
rm /home/cesi/Trying/archive/RTC_Micros.o
rm /home/cesi/Trying/archive/RTC_PCF8523.o
rm /home/cesi/Trying/archive/RTC_PCF8563.o
rm /home/cesi/Trying/archive/IPAddress.o
rm /home/cesi/Trying/archive/HID.o
rm /home/cesi/Trying/archive/EnvironmentCalulations.o
rm /home/cesi/Trying/archive/SPI.o
rm /home/cesi/Trying/archive/wiring_pulse.o
rm /home/cesi/Trying/archive/CDC.o
rm /home/cesi/Trying/archive/wiring_shift.o
rm /home/cesi/Trying/archive/WMath.o
rm /home/cesi/Trying/archive/Tone.o



##compilation du fichier source.cpp



avr-g++ -std=gnu++11 -c $1 -Os -DF_CPU=16000000UL -mmcu=atmega328p $LIB_PATHS -I/home/cesi/Trying/archive -I$path_pin -o/home/cesi/Trying/source.o

echo "c'est bon pour l'assemblage"



avr-g++ -Os -std=gnu++11 -o source.elf $LIB_PATHS -I$path_pin -mmcu=atmega328p -DF_CPU=16000000UL /home/cesi/Trying/Wire.o /home/cesi/Trying/twi.o /home/cesi/Trying/archive/*.o /home/cesi/Trying/source.o ##/home/cesi/Trying/archive/*.o


ls

echo "Passage du fichier .elf en hexadécimal"


avr-objcopy -j .text -j .data -O ihex source.elf source.hex

echo "Téléversement engagé"

avrdude -v -p atmega328p -c arduino -P /dev/ttyS0 -b 115200 -D -U flash:w:source.hex:i