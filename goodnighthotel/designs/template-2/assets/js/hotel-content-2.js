(function () {
  const booking = {
    fr: 'https://www.thebookingbutton.co.uk/properties/goodnightdirect/booking_widget?locale=fr',
    en: 'https://www.thebookingbutton.co.uk/properties/goodnightdirect/booking_widget?locale=en'
  };
  const copy = {
    fr: {
      nav: ['Découvrir', 'Chambres', 'Services', 'Contact'], cta: 'Réserver', language: 'EN', languageAria: 'Passer à l’anglais',
      hero: ['GOOD NIGHT', 'HÔTEL'], stats: [['64', 'Chambres'], ['2★', 'Hôtel 2 étoiles'], ['14:00', 'Arrivée']],
      advantagesTitle: 'Votre séjour <span class="text-[var(--hotel-green)]">à Arques</span>',
      advantagesDesc: 'Un hôtel pratique et accueillant pour dormir confortablement, prendre un petit déjeuner continental et découvrir le Pas-de-Calais.',
      advantages: [['Accueil simple', 'Une atmosphère conviviale et des informations claires pour votre séjour.'], ['Localisation', 'À proximité de Saint-Omer et avec un accès pratique à l’A26.'], ['Confort', 'Des chambres pour une à trois personnes, avec salle de bain privative, télévision et bureau.'], ['Services utiles', 'Wi-Fi gratuit, parking gratuit, petit déjeuner et arrivée tardive possible.']],
      testimonials: [['Une étape pratique à Arques.', '64 chambres, un parking gratuit et le Wi-Fi disponible dans les chambres.', 'Informations'], ['Un petit déjeuner continental.', 'Servi de 6 h 30 à 10 h du lundi au vendredi, et de 7 h 30 à 10 h 30 le week-end et les jours fériés.', 'Horaires'], ['Une base pour découvrir la région.', 'Saint-Omer, les marais audomarois et les sites locaux sont facilement accessibles.', 'À découvrir']],
      service: ['Chambres confortables', 'Petit déjeuner continental', 'Services essentiels'],
      faqTitle: 'Les informations pratiques',
      faq: [['Quels sont les horaires d’arrivée ?', 'Les chambres sont disponibles à partir de 14 h. Le départ doit être effectué avant 11 h 30. Une arrivée tardive est possible grâce au dispositif automatique.'], ['Quels sont les horaires du petit déjeuner ?', 'Le buffet continental est servi de 6 h 30 à 10 h du lundi au vendredi, et de 7 h 30 à 10 h 30 le samedi, le dimanche et les jours fériés.'], ['Le parking et le Wi-Fi sont-ils gratuits ?', 'Oui. Le parking privé sur place est gratuit et le Wi-Fi est disponible gratuitement dans les chambres.']],
      footer: ['Chambres', 'Services', 'Contact', 'Réserver'], footerDescription: 'Good Night Hôtel à Arques : 64 chambres, petit déjeuner continental, Wi-Fi et parking gratuits.', newsletter: 'Votre adresse e-mail', legal: 'Mentions légales'
    },
    en: {
      nav: ['Discover', 'Rooms', 'Services', 'Contact'], cta: 'Book now', language: 'FR', languageAria: 'Switch to French',
      hero: ['GOOD NIGHT', 'HOTEL'], stats: [['64', 'Rooms'], ['2★', 'Two-star hotel'], ['2:00 pm', 'Check-in']],
      advantagesTitle: 'Your stay <span class="text-[var(--hotel-green)]">in Arques</span>',
      advantagesDesc: 'A practical, welcoming hotel for a comfortable night, a continental breakfast and discovering the Pas-de-Calais region.',
      advantages: [['A warm welcome', 'A friendly atmosphere and clear information for your stay.'], ['A useful location', 'Close to Saint-Omer, with convenient access to the A26 motorway.'], ['Comfort', 'Rooms for one to three guests, with a private bathroom, television and desk.'], ['Useful services', 'Free Wi-Fi, free parking, breakfast and late arrival facilities.']],
      testimonials: [['A practical stop in Arques.', '64 rooms, free parking and Wi-Fi available in the rooms.', 'Information'], ['A continental breakfast.', 'Served from 6:30 to 10:00 am Monday to Friday, and from 7:30 to 10:30 am at weekends and on public holidays.', 'Opening hours'], ['A base for discovering the area.', 'Saint-Omer, the Audomarois marshes and local attractions are within easy reach.', 'Discover']],
      service: ['Comfortable rooms', 'Continental breakfast', 'Essential services'],
      faqTitle: 'Practical information',
      faq: [['What are the arrival times?', 'Rooms are available from 2:00 pm. Check-out is by 11:30 am. Late arrival is possible through the automatic arrival facility.'], ['When is breakfast served?', 'The continental buffet is served from 6:30 to 10:00 am Monday to Friday, and from 7:30 to 10:30 am on Saturdays, Sundays and public holidays.'], ['Are parking and Wi-Fi free?', 'Yes. Private on-site parking is free, and Wi-Fi is available free of charge in the rooms.']],
      footer: ['Rooms', 'Services', 'Contact', 'Book now'], footerDescription: 'Good Night Hotel in Arques: 64 rooms, continental breakfast, free Wi-Fi and free parking.', newsletter: 'Your email address', legal: 'Legal information'
    }
  };
  const q = (selector, root = document) => root.querySelector(selector);
  const qa = (selector, root = document) => Array.from(root.querySelectorAll(selector));
  const set = (node, value, html = false) => { if (!node) return; if (html) node.innerHTML = value; else node.textContent = value; };
  const addLanguageControls = () => {
    const pill = q('nav .glass-panel');
    if (pill && !q('#hotel-language-toggle')) { const button = document.createElement('button'); button.id = 'hotel-language-toggle'; button.className = 'rounded-full px-4 py-3 text-sm font-medium text-[var(--hotel-green)] hover:bg-[var(--hotel-cream)] transition-colors'; button.textContent = 'EN'; button.type = 'button'; pill.insertBefore(button, q('nav a[href="#contact"]')); }
    const overlay = q('#nav-links');
    if (overlay && !q('#hotel-language-mobile')) { const button = document.createElement('button'); button.id = 'hotel-language-mobile'; button.className = 'text-4xl md:text-6xl font-medium tracking-tighter hover:text-gray-500 transition-colors'; button.type = 'button'; button.textContent = 'English'; overlay.appendChild(button); }
  };
  const setLanguage = (language) => {
    const t = copy[language]; document.documentElement.lang = language; document.documentElement.dataset.language = language; addLanguageControls();
    set(q('nav .hidden.md\\:block'), 'GOOD NIGHT HÔTEL'); qa('#nav-links a').forEach((link, i) => set(link, t.nav[i] || t.nav[0])); set(q('nav a[href="#contact"]'), t.cta);
    set(q('#hotel-language-toggle'), t.language); set(q('#hotel-language-mobile'), language === 'fr' ? 'English' : 'Français'); q('#hotel-language-toggle')?.setAttribute('aria-label', t.languageAria);
    qa('header h1 span').forEach((span, i) => set(span, t.hero[i]));
    const kpis = qa('body > section')[0] ? qa('body > section')[0].querySelectorAll('.reveal-element') : []; t.stats.forEach((stat, i) => { const item = kpis[i]; if (item) { set(item.querySelector('.text-5xl'), stat[0]); set(item.querySelector('.text-lg'), stat[1]); } });
    const advantages = qa('body > section')[1]; if (advantages) { set(q('h2', advantages), t.advantagesTitle, true); set(q('p', advantages), t.advantagesDesc); qa('.group', advantages).forEach((card, i) => { const item = t.advantages[i]; if (!item) return; set(q('span', card), item[0]); set(q('p', card), item[1]); }); }
    const testimonials = q('#testimonials'); if (testimonials) qa('#slider-track > div', testimonials).forEach((card, i) => { const item = t.testimonials[i]; if (!item) return; const ps = qa('p', card); if (ps[0]) ps[0].textContent = item[0]; if (ps[1]) ps[1].textContent = item[1]; const spans = qa('span', card); if (spans[0]) spans[0].textContent = item[2]; if (spans[1]) spans[1].textContent = language === 'fr' ? 'Good Night Hôtel' : 'Good Night Hotel'; });
    set(q('#service-text'), t.service[0]); const faq = qa('body > section')[4]; if (faq) { set(q('h2', faq), t.faqTitle); qa('.accordion-item', faq).forEach((item, i) => { const pair = t.faq[i]; if (!pair) return; set(q('.accordion-header span', item), pair[0]); set(q('.accordion-content div', item), pair[1]); }); }
    qa('footer h4').forEach((heading, i) => set(heading, t.footer[i] || t.footer[0])); set(q('footer .space-y-6 p'), t.footerDescription); qa('footer input[type="email"]').forEach((input) => input.placeholder = t.newsletter); qa('footer a').filter((a) => /Datenschutz|Impressum|Privacy|Legal|Mentions/.test(a.textContent)).forEach((a) => set(a, t.legal));
    qa('a[href="#contact"]').forEach((link) => { link.href = booking[language]; link.target = '_blank'; link.rel = 'noopener'; }); localStorage.setItem('good-night-template-2-language', language);
  };
  addLanguageControls(); let language = localStorage.getItem('good-night-template-2-language') || 'fr'; setLanguage(language);
  document.addEventListener('click', (event) => { if (event.target.closest('#hotel-language-toggle, #hotel-language-mobile')) { language = language === 'fr' ? 'en' : 'fr'; setLanguage(language); } });
})();
