# MCU_STM32

Utilisation d'une carte STM32 L152RETx, projet MCU

## Contraintes et exigeances : 
- Utiliser un protocole de communication.
- Utiliser plusiseurs périphériques.
- Utiliser au moins une intéruption.

  Bonus : LPM, WWDOG

  ### Réalisé :
  Selecteur de mode via un bouton de la carte ( génération d'une intéruption ).
  4 modes : chronomètre, minuteur, alarme et heure, fuseaux horaires.
  Utilisation : ADC, PWM, afficheur 7 seg via SPI, RTC (sans temps réel), (Low power mode).

  #### Chronomètre :
  Un appui sur le bouton 1 de la carte ISEN rouge permet de lancer le chronomètre, le bouton 1 permet d'arrêter et de voir le temps réalisé.

  #### Minuteur :
  Paramétrage des secondes avec un potentiomètre utilisant l'ADC. Validation avec le bouton 1 (témoin led), puis paramétrage des minutes de façon similaires.
  Lorsque le bouton 2 est à nouveau pressé, un témoin led apparaît et le minuteur démarre.
  Quand le temps arrive à 0, le buzzer de la carte ISEN sonne (PWM).

  #### Alarme :
  Sans action de l'utilisateur, ce mode affiche l'heure.
  Pour paramétrer une alarme il suffit d'appuyer sur le bouton 2 (comme minuteur).
  Cette fois l'utilisateur est amené à parametrer les minutes et les heures et non les secondes.
  Une fois cette action faite, un appui sur le bouton 1 permet de valider d'abord le temps et donc de le lancer.
  Lorsque l'heure choisie sera l'heure courante, le moteur de la carte ISEN sonnera tant que l'utilisateur n'appuiera pas sur le bouton 2.

  #### Fuseaux horaires :
  Les fuseaux horaires ont été choisi arbitrairement. Ils sont activé si l'utilisateur appuie sur le bouton 1 dans le mode 4.
  Les acronymes des villes défilent alors sur l'afficheur 7 segments puis l'heure courante dans la ville s'affiche.

  Attention : Le module RTC est fonctionnel mais n'est pas relié à un module permettant de récupérer l'heure en temps réel.
  Par conséquent, l'heure est à changer dans le .ioc avant de téléverser le code dans le MCU. Les heures du mode 4 fuseaux horaires dépendent donc de cette heure selectionnée au préalable.

  Low Power Mode : Pour le moment, le code permet lors d'un appui sur le bouton 3 de mettre le MCU en mode sleep. La solution de réveil n'est quant à elle non fonctionnelle. Le bouton de reset peut cependant réveiller le MCU (l'heure de l'ioc redeviendra alors l'heure courante.
