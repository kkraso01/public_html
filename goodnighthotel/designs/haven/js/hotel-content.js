(function () {
  "use strict";

  var bookingUrl = "https://www.thebookingbutton.co.uk/properties/goodnightdirect/booking_widget";
  var copy = {
    fr: {
      title: "Good Night HÃ´tel â€” Arques",
      meta: "Good Night HÃ´tel Ã  Arques, dans le Pas-de-Calais : chambres confortables, petit-dÃ©jeuner et accueil chaleureux.",
      nav: ["Chambres", "Services", "Galerie", "Petit-dÃ©jeuner", "Avis", "Contact"],
      book: "RÃ©server",
      phone: "+33 3 21 93 81 20",
      heroEyebrow: "Arques Â· SÃ©jour chaleureux",
      heroTitle: "Dormez bien,<br>rÃ©veillez-vous ailleurs.",
      heroLead: "Un hÃ´tel accueillant au cÅ“ur de lâ€™Audomarois. Des chambres confortables, un petit-dÃ©jeuner gÃ©nÃ©reux et une Ã©quipe attentive Ã  chaque Ã©tape de votre sÃ©jour.",
      availability: "Voir les disponibilitÃ©s",
      explore: "DÃ©couvrir les chambres",
      checkin: "ArrivÃ©e", checkout: "DÃ©part", guests: "Voyageurs", guest: "voyageur", guestsPlural: "voyageurs",
      welcomeEyebrow: "Bienvenue au Good Night",
      welcomeTitle: "Une adresse simple, soignÃ©e et profondÃ©ment accueillante.",
      welcomeText: "Good Night HÃ´tel vous accueille Ã  Arques, Ã  quelques minutes de Saint-Omer. Installez-vous dans une chambre rÃ©novÃ©e et lumineuse, profitez dâ€™un petit-dÃ©jeuner convivial et retrouvez facilement les sites culturels, naturels et gourmands de la rÃ©gion.",
      est: "Depuis", estValue: "64", estText: "Ã€ Arques, prÃ¨s de Saint-Omer",
      stats: ["Chambres", "Ã€ deux pas de Saint-Omer", "Note des voyageurs"],
      roomsEyebrow: "Chambres & confort", roomsTitle: "Tout le nÃ©cessaire pour bien rÃ©cupÃ©rer", roomsNote: "Des chambres fonctionnelles et reposantes, pensÃ©es pour les sÃ©jours professionnels, les escapades en famille et les week-ends dans lâ€™Audomarois.",
      rooms: [
        ["Chambre rÃ©novÃ©e", "Ã€ partir de", "79 â‚¬", "/ nuit", "Une chambre lumineuse et confortable pour une nuit paisible, avec tout le nÃ©cessaire Ã  portÃ©e de main.", "1â€“2 personnes", "Douche", "Wi-Fi inclus", "RÃ©server"],
        ["Chambre Twin", "Ã€ partir de", "89 â‚¬", "/ nuit", "Deux lits sÃ©parÃ©s, un espace agrÃ©able et une solution pratique pour les amis, collÃ¨gues ou familles.", "2 personnes", "Lits sÃ©parÃ©s", "Petit-dÃ©jeuner", "RÃ©server"],
        ["Chambre familiale", "Ã€ partir de", "109 â‚¬", "/ nuit", "Un espace gÃ©nÃ©reux pour partager une Ã©tape confortable avant de dÃ©couvrir Saint-Omer et ses environs.", "3â€“4 personnes", "Espace familial", "Parking facile", "RÃ©server"]
      ],
      amenitiesEyebrow: "Les attentions qui comptent", amenitiesTitle: "Le confort, tout simplement",
      amenities: [["Petit-dÃ©jeuner", "Commencez la journÃ©e avec une sÃ©lection gÃ©nÃ©reuse de produits et de boissons chaudes."], ["Accueil attentif", "Une Ã©quipe disponible pour vous conseiller et rendre votre sÃ©jour plus agrÃ©able."], ["Produits locaux", "DÃ©couvrez les saveurs et les bonnes adresses de lâ€™Audomarois."], ["ArrivÃ©e facilitÃ©e", "Des informations claires pour arriver sereinement, mÃªme aprÃ¨s une journÃ©e bien remplie."], ["Wi-Fi", "Restez connectÃ© dans les chambres et les espaces communs."], ["Parking Ã  proximitÃ©", "Garez-vous facilement et partez explorer la rÃ©gion Ã  votre rythme."]],
      galleryEyebrow: "Ã€ dÃ©couvrir autour de nous", galleryTitle: "Des souvenirs Ã  quelques minutes", galleryNote: "Saint-Omer, la Coupole, les marais audomarois et les savoir-faire locaux : votre sÃ©jour commence ici.",
      diningEyebrow: "Petit-dÃ©jeuner & bonnes adresses", diningTitle: "Le matin prend son temps.", diningText: "Un petit-dÃ©jeuner simple et gÃ©nÃ©reux pour commencer la journÃ©e du bon pied, puis une Ã©quipe heureuse de vous recommander les restaurants, producteurs et sorties qui font le charme de la rÃ©gion.", menu: [["Petit-dÃ©jeuner continental", "Ã  partir de", "12 â‚¬"], ["Boisson chaude & viennoiserie", "Ã  partir de", "6 â‚¬"], ["Conseils restaurants", "pour nos hÃ´tes", "offerts"]], hours: "Petit-dÃ©jeuner 7 hâ€“10 h Â· Accueil selon vos besoins Â· Conseils toute la journÃ©e", diningCta: "Nous contacter",
      reviewsEyebrow: "Avis des voyageurs", reviewsTitle: "Un sÃ©jour qui fait du bien", reviewsCount: "4,6 sur 5 â€” avis vÃ©rifiÃ©s",
      reviews: [["Une Ã©quipe adorable et une chambre trÃ¨s propre. Lâ€™emplacement est parfait pour visiter Saint-Omer et les environs.", "Claire M.", "SÃ©jour en famille Â· France"], ["Accueil chaleureux, petit-dÃ©jeuner agrÃ©able et nuit trÃ¨s calme. Nous reviendrons avec plaisir.", "Thomas L.", "Escapade Â· France"], ["Une excellente Ã©tape professionnelle : tout est simple, confortable et bien organisÃ©.", "Sophie R.", "SÃ©jour professionnel Â· France"], ["TrÃ¨s bon rapport qualitÃ©-prix et de bons conseils pour dÃ©couvrir lâ€™Audomarois.", "Marc D.", "Week-end Â· Belgique"]],
      ctaEyebrow: "Ã€ bientÃ´t Ã  Arques", ctaTitle: "Votre sÃ©jour commence par un bonjour.", ctaText: "Consultez les disponibilitÃ©s et prÃ©parez votre prochaine Ã©tape dans le Pas-de-Calais.", contactEyebrow: "Nous trouver", contactTitle: "Arques, au cÅ“ur de lâ€™Audomarois", contactText: "Une base pratique pour explorer Saint-Omer, les marais, la Coupole et les nombreuses expÃ©riences de la rÃ©gion.", addressLabel: "Adresse", address: "45 Rue Jean Baptiste Colbert, 62510 Arques", reception: "Accueil", receptionText: "Une Ã©quipe Ã  votre Ã©coute Â· ArrivÃ©es selon rÃ©servation", name: "Votre nom", message: "Dates, type de chambre, demande particuliÃ¨reâ€¦", send: "Envoyer la demande", footerText: "Un hÃ´tel chaleureux Ã  Arques, prÃ¨s de Saint-Omer. Dormez bien, explorez davantage.", footerLinks: ["Chambres & confort", "Services", "Galerie", "Petit-dÃ©jeuner", "Avis"], footerPlace: "Arques Â· Pas-de-Calais", newsletter: "Recevez nos nouvelles et nos idÃ©es de sÃ©jour.", subscribe: "Sâ€™inscrire", rights: "Â© 2026 Good Night HÃ´tel. Tous droits rÃ©servÃ©s."
    },
    en: {
      title: "Good Night Hotel â€” Arques",
      meta: "Good Night Hotel in Arques, Pas-de-Calais: comfortable rooms, breakfast and a warm welcome.",
      nav: ["Rooms", "Amenities", "Gallery", "Breakfast", "Reviews", "Contact"], book: "Book a stay", phone: "+33 3 21 93 81 20",
      heroEyebrow: "Arques Â· A warm welcome", heroTitle: "Sleep well,<br>wake up somewhere new.", heroLead: "A welcoming hotel in the heart of the Audomarois. Comfortable rooms, a generous breakfast and an attentive team for every part of your stay.", availability: "Check availability", explore: "Explore the rooms", checkin: "Check-in", checkout: "Check-out", guests: "Guests", guest: "guest", guestsPlural: "guests",
      welcomeEyebrow: "Welcome to Good Night", welcomeTitle: "A simple, thoughtful and genuinely welcoming address.", welcomeText: "Good Night Hotel welcomes you to Arques, just minutes from Saint-Omer. Settle into a bright renovated room, enjoy a friendly breakfast and reach the regionâ€™s cultural, natural and culinary highlights with ease.", est: "Since", estValue: "64", estText: "In Arques, near Saint-Omer", stats: ["Rooms", "Close to Saint-Omer", "Guest rating"], roomsEyebrow: "Rooms & comfort", roomsTitle: "Everything you need to rest well", roomsNote: "Comfortable, practical rooms for business trips, family stays and weekends exploring the Audomarois.", rooms: [["Renovated Room", "From", "â‚¬79", "/ night", "A bright and comfortable room for a peaceful night, with everything you need close at hand.", "1â€“2 guests", "Shower", "Wi-Fi included", "Book now"], ["Twin Room", "From", "â‚¬89", "/ night", "Two separate beds and a welcoming space for friends, colleagues or family.", "2 guests", "Separate beds", "Breakfast", "Book now"], ["Family Room", "From", "â‚¬109", "/ night", "A generous space for a comfortable stop before discovering Saint-Omer and the surrounding area.", "3â€“4 guests", "Family space", "Easy parking", "Book now"]], amenitiesEyebrow: "The details that matter", amenitiesTitle: "Comfort, simply done", amenities: [["Breakfast", "Start the day with a generous selection of food and hot drinks."], ["A warm welcome", "A helpful team ready to make your stay easier and more enjoyable."], ["Local discoveries", "Find the flavours and addresses that make the Audomarois special."], ["Easy arrival", "Clear information so you can arrive calmly, even after a long day."], ["Wi-Fi", "Stay connected in your room and throughout the common areas."], ["Nearby parking", "Park easily and explore the region at your own pace."]], galleryEyebrow: "Discover the area", galleryTitle: "Memories just minutes away", galleryNote: "Saint-Omer, La Coupole, the Audomarois marshes and local craftsmanship: your stay starts here.", diningEyebrow: "Breakfast & local recommendations", diningTitle: "Mornings take their time.", diningText: "A generous, uncomplicated breakfast to start the day well, followed by a team happy to recommend the restaurants, producers and experiences that make the area special.", menu: [["Continental breakfast", "from", "â‚¬12"], ["Hot drink & pastry", "from", "â‚¬6"], ["Restaurant recommendations", "for our guests", "free"]], hours: "Breakfast 7â€“10 am Â· Welcoming you around your booking Â· Local advice all day", diningCta: "Contact us", reviewsEyebrow: "Guest reviews", reviewsTitle: "A stay that does you good", reviewsCount: "4.6 out of 5 â€” verified reviews", reviews: [["A lovely team and an exceptionally clean room. Perfectly placed for visiting Saint-Omer and the surrounding area.", "Claire M.", "Family stay Â· France"], ["A warm welcome, a pleasant breakfast and a very quiet night. We will happily return.", "Thomas L.", "Getaway Â· France"], ["An excellent business stop: everything is simple, comfortable and well organised.", "Sophie R.", "Business stay Â· France"], ["Great value and excellent advice for discovering the Audomarois.", "Marc D.", "Weekend Â· Belgium"]], ctaEyebrow: "See you in Arques", ctaTitle: "Your stay starts with a warm welcome.", ctaText: "Check availability and plan your next stop in Pas-de-Calais.", contactEyebrow: "Find us", contactTitle: "Arques, in the heart of the Audomarois", contactText: "A practical base for exploring Saint-Omer, the marshes, La Coupole and the regionâ€™s many experiences.", addressLabel: "Address", address: "45 Rue Jean Baptiste Colbert, 62510 Arques", reception: "Reception", receptionText: "A team here to help Â· Arrivals according to your booking", name: "Your name", message: "Dates, room type, a special requestâ€¦", send: "Send enquiry", footerText: "A welcoming hotel in Arques, near Saint-Omer. Sleep well, explore more.", footerLinks: ["Rooms & comfort", "Amenities", "Gallery", "Breakfast", "Reviews"], footerPlace: "Arques Â· Pas-de-Calais", newsletter: "Receive our news and local stay ideas.", subscribe: "Subscribe", rights: "Â© 2026 Good Night Hotel. All rights reserved."
    }
  };

  function all(sel) { return Array.prototype.slice.call(document.querySelectorAll(sel)); }
  function setText(sel, value, index) { var nodes = all(sel); if (nodes[index || 0]) nodes[index || 0].innerHTML = value; }
  function setMany(sel, values) { var nodes = all(sel); if (!Array.isArray(values)) values = nodes.map(function () { return values; }); nodes.forEach(function (node, i) { if (values[i] !== undefined) node.innerHTML = values[i]; }); }
  function setAttr(sel, attr, value, index) { var nodes = all(sel); if (nodes[index || 0]) nodes[index || 0].setAttribute(attr, value); }

  function render(lang) {
    var d = copy[lang];
    document.documentElement.lang = lang;
    document.title = d.title;
    var meta = document.querySelector('meta[name="description"]'); if (meta) meta.content = d.meta;
    setText(".brand-word", "Good Night"); setAttr(".brand", "aria-label", d.title); setAttr(".brand--footer", "aria-label", d.title);
    setMany(".nav-list a", d.nav); setMany(".btn-amber", d.book); setMany(".header-book", d.book); setMany(".nav-cta", d.book);
    setText(".header-phone", d.phone); setAttr(".header-phone", "href", "tel:+33321938120");
    setText(".hero .eyebrow", d.heroEyebrow); setText(".hero-title", d.heroTitle); setText(".hero-lead", d.heroLead); setText(".hero-cta-row .btn-amber", d.availability); setText(".hero-cta-row .btn-ghost-light", d.explore);
    setMany(".bw-field label", [d.checkin, d.checkout, d.guests]); setText(".bw-submit", d.availability);
    setText(".welcome .eyebrow", d.welcomeEyebrow); setText(".welcome-copy .section-title", d.welcomeTitle); setText(".welcome-copy > p:not(.eyebrow)", d.welcomeText); setText(".wb-num", d.est + "<br>" + d.estValue); setText(".wb-txt", d.estText); setMany(".stat-label", d.stats); setMany(".stat-num", lang === "fr" ? ["64", "Près de Saint-Omer", "Hôtel 2 étoiles"] : ["64", "Près de Saint-Omer", "Hôtel 2 étoiles"]);
    setText("#rooms .eyebrow", d.roomsEyebrow); setText("#rooms .section-title", d.roomsTitle); setText("#rooms .section-head-note", d.roomsNote);
    all(".room-card").forEach(function (card, i) { var r = d.rooms[i]; if (!r) return; setText(".room-name", r[0], i); setText(".room-price", '<span>' + r[1] + '</span> ' + r[2] + ' <small>' + r[3] + '</small>', i); setText(".room-desc", r[4], i); setMany(".room-specs", []); var specs = card.querySelectorAll(".room-specs li"); [r[5], r[6], r[7]].forEach(function (x, j) { if (specs[j]) specs[j].lastChild.textContent = " " + x; }); var link = card.querySelector(".room-link"); if (link) link.innerHTML = r[8] + " <span aria-hidden=\"true\">â†’</span>"; });
    setText("#amenities .eyebrow", d.amenitiesEyebrow); setText("#amenities .section-title", d.amenitiesTitle); all(".amenity").forEach(function (node, i) { if (d.amenities[i]) { setText("h3", d.amenities[i][0], i); setText("p", d.amenities[i][1], i); } });
    setText("#gallery .eyebrow", d.galleryEyebrow); setText("#gallery .section-title", d.galleryTitle); setText("#gallery .section-head-note", d.galleryNote);
    setText("#dining .eyebrow", d.diningEyebrow); setText("#dining .section-title", d.diningTitle); setText(".dining-copy > p:not(.eyebrow)", d.diningText); setMany(".dm-name", d.menu.map(function (x) { return x[0]; })); setMany(".dm-price", d.menu.map(function (x) { return x[2]; })); setText(".dining-hours", d.hours); setText("#dining .btn-amber", d.diningCta);
    setText("#reviews .eyebrow", d.reviewsEyebrow); setText("#reviews .section-title", d.reviewsTitle); setText("#reviews .section-head-note", d.reviewsCount); all(".review-card").forEach(function (card, i) { if (!d.reviews[i]) return; setText("blockquote", 'â€œ' + d.reviews[i][0] + 'â€', i); setText(".reviewer strong", d.reviews[i][1], i); setText(".reviewer small", d.reviews[i][2], i); });
    setText(".cta-band .eyebrow", d.ctaEyebrow); setText(".cta-title", d.ctaTitle); setText(".cta-sub", d.ctaText); setText(".cta-call", d.phone); setAttr(".cta-call", "href", "tel:+33321938120");
    setText("#contact .eyebrow", d.contactEyebrow); setText("#contact .section-title", d.contactTitle); setText(".contact-intro", d.contactText); setMany(".contact-list strong", [d.addressLabel, "Téléphone", "Email", d.reception]); setText(".contact-list li:nth-child(1) span:last-child", '<strong>' + d.addressLabel + '</strong>' + d.address); setText(".contact-list li:nth-child(4) span:last-child", '<strong>' + d.reception + '</strong>' + d.receptionText); setAttr("#ef-name", "placeholder", d.name); setAttr("#ef-msg", "placeholder", d.message); setText("#enquiryForm .btn-amber", d.send); setText(".map-pin span:last-child", '<strong>Good Night Hôtel</strong>45 Rue Jean Baptiste Colbert'); setText(".footer-brand > p", d.footerText); setMany(".footer-col:first-of-type a", d.footerLinks); setText(".footer-col:nth-of-type(2) li", d.footerPlace); setText(".footer-news p", d.newsletter); setAttr("#news-email", "placeholder", "you@email.com"); setText("#newsForm .btn-amber", d.subscribe); setText(".footer-bottom p", d.rights);
    var toggle = document.getElementById("languageToggle"); if (toggle) toggle.textContent = lang === "fr" ? "EN" : "FR";
    all(".gallery-item").forEach(function (node, i) { var labels = lang === "fr" ? ["Chambre Good Night", "Petit-dÃ©jeuner", "Saint-Omer", "La Coupole", "Les marais audomarois", "Une escapade dans la rÃ©gion"] : ["Good Night room", "Breakfast", "Saint-Omer", "La Coupole", "The Audomarois marshes", "A getaway in the region"]; node.setAttribute("data-caption", labels[i]); });
    var images = ["hotel-assets/hotel/hero.jpg", "hotel-assets/rooms/img_hotel.jpg", "hotel-assets/rooms/chambre-renov%C3%A9e.png", "hotel-assets/rooms/chambre-twin-renov%C3%A9e.png", "hotel-assets/rooms/check-in.jpg", "hotel-assets/tourism/ArcInternational.jpg", "hotel-assets/tourism/bateau-promenade-Copie.jpg", "hotel-assets/tourism/coupole.jpg", "hotel-assets/tourism/Planetarium.jpg", "hotel-assets/tourism/rando-rail.jpg", "hotel-assets/tourism/carre-saint-martin.jpg", "hotel-assets/rooms/img_breakfast.jpg", "hotel-assets/rooms/img_hotel.jpg", "hotel-assets/tourism/coupole.jpg"];
    var imgNodes = all("img");
    if (imgNodes[0]) imgNodes[0].src = images[0];
    if (imgNodes[1]) imgNodes[1].src = images[1];
    all(".room-media img").forEach(function (node, i) { node.src = images[2 + i]; });
    all(".gallery-item").forEach(function (node, i) { var src = images[5 + i]; node.querySelector("img").src = src; node.setAttribute("data-full", src); });
    var diningImages = all(".dining-media img"); diningImages.forEach(function (node, i) { node.src = images[11 + i]; });
    var mapImage = document.querySelector(".frame-map img"); if (mapImage) mapImage.src = "hotel-assets/tourism/carre-saint-martin.jpg";
  }

  function initLanguage() {
    var lang = localStorage.getItem("good-night-haven-language") || "fr";
    var button = document.createElement("button"); button.id = "languageToggle"; button.type = "button"; button.className = "language-toggle"; button.setAttribute("aria-label", "Change language");
    var host = document.querySelector(".header-actions"); if (host) host.insertBefore(button, host.firstChild);
    button.addEventListener("click", function () { lang = lang === "fr" ? "en" : "fr"; localStorage.setItem("good-night-haven-language", lang); render(lang); });
    render(lang);
  }
  document.addEventListener("DOMContentLoaded", initLanguage);
})();




