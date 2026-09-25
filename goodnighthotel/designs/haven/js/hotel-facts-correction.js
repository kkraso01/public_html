(function () {
  const fr = {
    rooms: ['Chambre classique', 'Chambre twin', 'Chambre triple'],
    roomFacts: ['1–3 personnes · salle de bain privative · Wi-Fi gratuit', '2 personnes · lits séparés · Wi-Fi gratuit', '3 personnes · salle de bain privative · parking gratuit'],
    breakfast: ['6 h 30–10 h', '6 h 30–10 h', '6 h 30–10 h'],
    reviews: ['64 chambres à Arques.', 'Parking privé gratuit sur place.', 'Arrivée possible grâce au dispositif automatique.', 'Animaux acceptés sous réserve des conditions et du supplément applicables.']
  };
  const en = {
    rooms: ['Classic Room', 'Twin Room', 'Triple Room'],
    roomFacts: ['1–3 guests · private bathroom · free Wi-Fi', '2 guests · separate beds · free Wi-Fi', '3 guests · private bathroom · free parking'],
    breakfast: ['6:30–10:00 am', '6:30–10:00 am', '6:30–10:00 am'],
    reviews: ['64 rooms in Arques.', 'Free private on-site parking.', 'Late arrival is possible through the automatic arrival facility.', 'Pets are accepted, subject to applicable conditions and supplement.']
  };
  const all = (s) => Array.from(document.querySelectorAll(s));
  const set = (s, value, i) => { const n = all(s)[i || 0]; if (n) n.textContent = value; };
  function correct() {
    const d = document.documentElement.lang === 'en' ? en : fr;
    set('.header-phone', '+33 3 21 93 81 20'); set('.cta-call', '+33 3 21 93 81 20');
    set('.wb-num', d === en ? 'Rooms\n64' : 'Chambres\n64');
    all('.stat-num').forEach((n, i) => { n.textContent = ['64', '2★', d === en ? '2 pm' : '14 h'][i] || n.textContent; });
    all('.room-card').forEach((card, i) => { set('.room-name', d.rooms[i], i); const price = card.querySelector('.room-price'); if (price) price.textContent = d === en ? 'Availability' : 'Disponibilité'; const description = card.querySelector('.room-desc'); if (description) description.textContent = d.roomFacts[i]; });
    all('.dm-price').forEach((n, i) => { n.textContent = i === 0 ? (d === en ? '6:30–10:00 am / 7:30–10:30 am' : '6 h 30–10 h / 7 h 30–10 h 30') : (d === en ? 'On request' : 'Sur demande'); });
    all('.review-card').forEach((card, i) => { const quote = card.querySelector('blockquote'); if (quote) quote.textContent = d.reviews[i] || ''; const name = card.querySelector('.reviewer strong'); if (name) name.textContent = d === en ? 'Hotel information' : 'Information de l’hôtel'; });
    const address = document.querySelector('.contact-list li:nth-child(1) span:last-child'); if (address) address.innerHTML = '<strong>' + (d === en ? 'Address' : 'Adresse') + '</strong>45 Rue Jean Baptiste Colbert<br>62510 Arques<br>France';
    const map = document.querySelector('.map-pin span:last-child'); if (map) map.innerHTML = '<strong>Good Night Hôtel</strong>45 Rue Jean Baptiste Colbert, 62510 Arques';
    const copyright = document.querySelector('.footer-bottom p'); if (copyright) copyright.textContent = d === en ? '© 2026 Good Night Hotel. All rights reserved.' : '© 2026 Good Night Hôtel. Tous droits réservés.';
  }
  window.setTimeout(correct, 80);
  document.addEventListener('click', (event) => { if (event.target.closest('#languageToggle')) window.setTimeout(correct, 80); });
})();
