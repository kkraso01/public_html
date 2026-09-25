(function () {
  const text = (selector, value) => {
    const node = document.querySelector(selector);
    if (node) node.textContent = value;
  };
  const html = (selector, value) => {
    const node = document.querySelector(selector);
    if (node) node.innerHTML = value;
  };

  text('.nav-logo', 'GOOD NIGHT HÔTEL');
  document.querySelectorAll('.nav-links .nav-link').forEach((link, index) => {
    const labels = ['L’expérience', 'Services', 'Chambres', 'Contact'];
    link.textContent = labels[index] || link.textContent;
    link.setAttribute('href', ['#philosophy', '#services', '#layouts', '#contact'][index] || '#');
  });
  const navCta = document.querySelector('.nav-cta span:last-child');
  if (navCta) navCta.textContent = 'Réserver';

  text('.hero-tag', 'Hôtel chaleureux à Arques');
  html('.hero-title', 'Une pause <em style="filter:drop-shadow(0 5px 12px rgba(0,0,0,.8))">bien pensée</em><br>au cœur du Pas-de-Calais');
  html('.hero-description', 'Good Night Hôtel — un lieu simple et accueillant pour dormir confortablement, prendre le temps et découvrir la région.<br>Chambres, petit déjeuner et services essentiels pour un séjour sans complication.');
  document.querySelectorAll('.hero-stat').forEach((stat, index) => {
    const values = [['64', 'Chambres'], ['2★', 'Hôtel 2 étoiles'], ['14 h', 'Arrivée']][index];
    if (!values) return;
    text('.hero-stat-number', values[0]);
    text('.hero-stat-label', values[1]);
  });

  text('#philosophy .section-label', 'Notre philosophie');
  html('#philosophy .section-title', 'Le confort comme <em>ligne directrice</em>');
  html('.philosophy-quote', '« L’essentiel, c’est un <span>accueil sincère</span> et une nuit vraiment reposante. »');
  text('.philosophy-desc', 'Good Night Hôtel est une adresse pratique et chaleureuse à Arques, pensée pour les voyageurs de passage comme pour les escapades en famille.');
  document.querySelectorAll('.philosophy-feature').forEach((feature, index) => {
    const items = [
      ['Une atmosphère apaisée', 'Des espaces simples où l’on se sent immédiatement bien.'],
      ['Une base idéale', 'À proximité de Saint-Omer et des lieux touristiques.'],
      ['Une découverte locale', 'Des expériences à vivre autour de l’hôtel.']
    ][index];
    if (!items) return;
    const title = feature.querySelector('strong, b');
    const body = feature.querySelector('.philosophy-feature-text');
    if (title) title.textContent = items[0];
    if (body) body.textContent = items[1];
  });

  text('#services .services-header', 'Tout ce qu’il faut pour profiter d’un séjour idéal à Arques.');
  document.querySelectorAll('#services .service-card').forEach((card, index) => {
    const items = ['Wifi gratuit dans les chambres.', 'Parking gratuit sur place.', 'Buffet continental chaque matin.', 'Arrivée tardive grâce au guichet automatique.', 'Lits bébé et équipements famille disponibles.', 'Une localisation pratique près de Saint-Omer.'];
    const icon = card.querySelector('iconify-icon');
    card.innerHTML = `${icon ? icon.outerHTML : ''}<span class="service-card-number">0${index + 1}</span><p>${items[index] || items[0]}</p>`;
  });

  text('#projects .section-label', 'Chambres & petit déjeuner');
  html('#projects .section-title', 'Choisissez votre <em>point de départ</em>');
  text('#projects .projects-subtitle', '64 chambres avec salle de bain privative, télévision, espace bureau et wifi gratuit.');
  document.querySelectorAll('#projects .project-card').forEach((card, index) => {
    const items = ['Chambre classique, un espace confortable pour se reposer.', 'Chambre twin, pratique pour les séjours à deux.', 'Buffet continental, pour commencer la journée en douceur.'];
    card.textContent = items[index] || items[0];
    card.setAttribute('href', '#contact');
  });
  const projectButton = document.querySelector('#projects .btn');
  if (projectButton) projectButton.querySelector('span') && (projectButton.querySelector('span').textContent = 'Voir les chambres');

  text('#process .section-label', 'Services');
  html('#process .section-title', 'Un séjour <em>sans complication</em>');
  document.querySelectorAll('.process-step').forEach((step, index) => {
    step.textContent = ['Réservez directement votre chambre.', 'Profitez d’un accueil simple et de services utiles.', 'Commencez la journée avec notre buffet continental.', 'Découvrez Arques, Saint-Omer et les environs.'][index] || step.textContent;
  });

  html('#layouts .section-title', 'Des chambres pour chaque <em>séjour</em>');
  const layouts = ['Chambre classique et confortable.', 'Chambre twin pour deux voyageurs.', 'Chambre triple pour les familles.', 'Salle de bain privative avec douche.', 'Télévision et espace bureau.', 'Wifi gratuit dans les chambres.', 'Petit déjeuner continental.', 'Arrivée tardive possible.', '64 chambres au total, à Arques.'];
  document.querySelectorAll('#layouts .grid > div').forEach((card, index) => {
    if (index === 8) {
      card.querySelector('.bg-\\[var\\(--color-sage\\)\\]')?.replaceChildren(document.createTextNode('À découvrir'));
      const image = card.querySelector('img');
      if (image) { image.src = 'assets/images/rooms/chambre-twin-renov%C3%A9e.png'; image.alt = 'Chambre twin rénovée'; }
      const heading = card.querySelector('h3'); if (heading) heading.textContent = 'Chambre twin';
      const paragraph = card.querySelector('p'); if (paragraph) paragraph.textContent = 'Une chambre lumineuse et pratique pour un séjour confortable.';
      card.querySelectorAll('span').forEach((span, i) => { if (i === 0) span.textContent = '45, Rue Jean Baptiste Colbert'; if (i === 1) span.textContent = '62510 Arques'; if (i === 2) span.textContent = '03.21.93.81.20'; });
    } else card.textContent = layouts[index] || layouts[0];
  });

  text('#journal .section-label', 'L’hôtel');
  html('#journal .section-title', 'Un accueil <em>essentiel</em>');
  document.querySelectorAll('.blog-card').forEach((card, index) => { card.textContent = ['Des chambres équipées pour dormir et travailler confortablement.', 'Un buffet continental avec boissons chaudes, pains, viennoiseries et produits frais.', 'Un hôtel pratique, accessible et proche des lieux touristiques.'][index] || ''; });
  const journalButton = document.querySelector('#journal .btn'); if (journalButton) journalButton.textContent = 'Découvrir l’hôtel';

  text('#contact .section-label', 'Votre prochaine étape');
  html('#contact .cta-title', 'Réservez votre <em>séjour</em>');
  text('#contact .cta-text', 'Laissez vos coordonnées pour être recontacté par l’hôtel ou utilisez le bouton de réservation pour vérifier les disponibilités.');
  const inputs = document.querySelectorAll('#contact input'); if (inputs[0]) inputs[0].placeholder = 'Votre nom'; if (inputs[1]) inputs[1].placeholder = 'Votre téléphone'; if (inputs[2]) inputs[2].placeholder = 'Votre email';
  const ctaButton = document.querySelector('#contact button'); if (ctaButton) ctaButton.querySelector('span') && (ctaButton.querySelector('span').textContent = 'Être recontacté');

  text('.footer-logo', 'GOOD NIGHT HÔTEL');
  document.querySelectorAll('.footer-column h4').forEach((heading, index) => { heading.textContent = ['Menu', 'Contacts', 'Réserver'][index] || heading.textContent; });
  const footerBrand = document.querySelector('.footer-brand p'); if (footerBrand) footerBrand.textContent = 'Un hôtel chaleureux à Arques, dans le Pas-de-Calais. Chambres, petit déjeuner et services essentiels.';
  document.querySelectorAll('.footer-column ul').forEach((list, index) => { if (index === 0) list.innerHTML = '<li><a href="#philosophy">L’expérience</a></li><li><a href="#services">Services</a></li><li><a href="#layouts">Chambres</a></li><li><a href="#contact">Contact</a></li>'; if (index === 1) list.innerHTML = '<li><a href="mailto:sales@good-night-hotel.com">sales@good-night-hotel.com</a></li><li><a href="tel:+33321938120">03.21.93.81.20</a></li><li>45, Rue Jean Baptiste Colbert<br>62510 Arques</li>'; });
  document.title = 'Good Night Hôtel — une pause bien pensée à Arques';
})();

(function () {
  const translations = {
    fr: {
      nav: ['L’expérience', 'Services', 'Chambres', 'Contact'],
      cta: 'Réserver',
      tag: 'Hôtel chaleureux à Arques',
      title: 'Une pause <em>bien pensée</em><br>au cœur du Pas-de-Calais',
      description: 'Good Night Hôtel — un lieu simple et accueillant pour dormir confortablement, prendre le temps et découvrir la région.<br>Chambres, petit déjeuner et services essentiels pour un séjour sans complication.',
      philosophyLabel: 'Notre philosophie',
      philosophyTitle: 'Le confort comme <em>ligne directrice</em>',
      philosophyQuote: '« L’essentiel, c’est un <span>accueil sincère</span> et une nuit vraiment reposante. »',
      philosophyDesc: 'Good Night Hôtel est une adresse pratique et chaleureuse à Arques, pensée pour les voyageurs de passage comme pour les escapades en famille.',
      servicesLabel: 'Infrastructures',
      servicesHeader: 'Tout ce qu’il faut pour profiter d’un séjour idéal à Arques.',
      projectLabel: 'Chambres & petit déjeuner',
      projectTitle: 'Choisissez votre <em>point de départ</em>',
      projectSubtitle: '64 chambres avec salle de bain privative, télévision, espace bureau et wifi gratuit.',
      processLabel: 'Services',
      processTitle: 'Un séjour <em>sans complication</em>',
      layoutTitle: 'Des chambres pour chaque <em>séjour</em>',
      journalLabel: 'L’hôtel',
      journalTitle: 'Un accueil <em>essentiel</em>',
      contactLabel: 'Votre prochaine étape',
      contactTitle: 'Réservez votre <em>séjour</em>',
      contactText: 'Laissez vos coordonnées pour être recontacté par l’hôtel ou utilisez le bouton de réservation pour vérifier les disponibilités.',
      contactButton: 'Être recontacté',
      footerMenu: 'Menu', footerContact: 'Contacts', footerBook: 'Réserver',
      footerDescription: 'Un hôtel chaleureux à Arques, dans le Pas-de-Calais. Chambres, petit déjeuner et services essentiels.',
      navAria: 'Passer à l’anglais', button: 'EN'
    },
    en: {
      nav: ['Experience', 'Services', 'Rooms', 'Contact'],
      cta: 'Book now',
      tag: 'A welcoming hotel in Arques',
      title: 'A well-considered <em>pause</em><br>in the Pas-de-Calais',
      description: 'Good Night Hotel — a simple, welcoming place to sleep comfortably, slow down and discover the region.<br>Rooms, breakfast and essential services for an uncomplicated stay.',
      philosophyLabel: 'Our philosophy',
      philosophyTitle: 'Comfort as our <em>guiding principle</em>',
      philosophyQuote: '“The essential thing is a <span>warm welcome</span> and a truly restful night.”',
      philosophyDesc: 'Good Night Hotel is a practical, welcoming address in Arques, designed for passing travellers and family getaways alike.',
      servicesLabel: 'Facilities',
      servicesHeader: 'Everything you need for an ideal stay in Arques.',
      projectLabel: 'Rooms & breakfast',
      projectTitle: 'Choose your <em>starting point</em>',
      projectSubtitle: '64 rooms with private bathrooms, television, desk space and free Wi-Fi.',
      processLabel: 'Services',
      processTitle: 'An <em>uncomplicated</em> stay',
      layoutTitle: 'Rooms for every <em>kind of stay</em>',
      journalLabel: 'The hotel',
      journalTitle: 'A <em>thoughtful</em> welcome',
      contactLabel: 'Your next step',
      contactTitle: 'Book your <em>stay</em>',
      contactText: 'Leave your details and the hotel will contact you, or use the booking button to check availability.',
      contactButton: 'Contact me',
      footerMenu: 'Menu', footerContact: 'Contact', footerBook: 'Book now',
      footerDescription: 'A welcoming hotel in Arques, Pas-de-Calais. Rooms, breakfast and essential services.',
      navAria: 'Switch to French', button: 'FR'
    }
  };

  const setHTML = (selector, value) => { const node = document.querySelector(selector); if (node) node.innerHTML = value; };
  const setText = (selector, value) => { const node = document.querySelector(selector); if (node) node.textContent = value; };
  const setLanguage = (language) => {
    const t = translations[language];
    document.documentElement.lang = language;
    document.documentElement.dataset.language = language;
    document.querySelectorAll('.nav-links .nav-link, .nav-mobile-overlay .nav-link').forEach((link, index) => { if (t.nav[index]) link.textContent = t.nav[index]; });
    document.querySelectorAll('.nav-cta span').forEach((node) => { node.textContent = t.cta; });
    document.querySelectorAll('#languageToggle, .language-toggle').forEach((button) => { button.textContent = t.button; button.setAttribute('aria-label', t.navAria); });
    setText('.hero-tag', t.tag); setHTML('.hero-title', t.title); setHTML('.hero-description', t.description);
    setText('#philosophy .section-label', t.philosophyLabel); setHTML('#philosophy .section-title', t.philosophyTitle); setHTML('.philosophy-quote', t.philosophyQuote); setText('.philosophy-desc', t.philosophyDesc);
    setText('#services .section-label', t.servicesLabel); setText('#services .services-header', t.servicesHeader);
    setText('#projects .section-label', t.projectLabel); setHTML('#projects .section-title', t.projectTitle); setText('#projects .projects-subtitle', t.projectSubtitle);
    setText('#process .section-label', t.processLabel); setHTML('#process .section-title', t.processTitle); setHTML('#layouts .section-title', t.layoutTitle);
    setText('#journal .section-label', t.journalLabel); setHTML('#journal .section-title', t.journalTitle); setText('#contact .section-label', t.contactLabel); setHTML('#contact .cta-title', t.contactTitle); setText('#contact .cta-text', t.contactText);
    document.querySelectorAll('#contact button').forEach((button) => { const span = button.querySelector('span'); if (span) span.textContent = t.contactButton; });
    const footerHeadings = document.querySelectorAll('.footer-column h4'); if (footerHeadings[0]) footerHeadings[0].textContent = t.footerMenu; if (footerHeadings[1]) footerHeadings[1].textContent = t.footerContact; if (footerHeadings[2]) footerHeadings[2].textContent = t.footerBook;
    setText('.footer-brand p', t.footerDescription);
    localStorage.setItem('good-night-language', language);
  };

  let language = localStorage.getItem('good-night-language') || 'fr';
  setLanguage(language);
  document.querySelectorAll('#languageToggle, .language-toggle').forEach((button) => button.addEventListener('click', () => { language = language === 'fr' ? 'en' : 'fr'; setLanguage(language); }));

  // The pasted reference was exported with several UTF-8 strings decoded as Latin-1.
  // Repair remaining visible text and accessibility attributes after the template loads.
  const repair = (value) => {
    if (!/[ÃÂâÐÑ]/.test(value)) return value;
    try { return decodeURIComponent(escape(value)); } catch (_) { return value; }
  };
  const walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
  const textNodes = [];
  while (walker.nextNode()) textNodes.push(walker.currentNode);
  textNodes.forEach((node) => { node.nodeValue = repair(node.nodeValue); });
  document.querySelectorAll('[alt],[title],[placeholder],[aria-label]').forEach((node) => ['alt', 'title', 'placeholder', 'aria-label'].forEach((attribute) => { if (node.hasAttribute(attribute)) node.setAttribute(attribute, repair(node.getAttribute(attribute))); }));
})();
