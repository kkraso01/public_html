(function () {
  var translations = {
    'Book a room': 'Réserver une chambre',
    'Where silence speaks of splendour.': 'Un hôtel simple et accueillant à Arques.',
    'Rooms & Breakfast': 'Chambres et petit-déjeuner',
    'See all rooms': 'Voir les chambres',
    'About Good Night Hotel': 'À propos de Good Night Hôtel',
    'Services & Area': 'Services et alentours',
    'Contact': 'Contact',
    'Home': 'Accueil',
    'Offers': 'Offres',
    'A comfortable base for your next stay.': 'Un point de départ pratique pour votre séjour.',
    'Discover the flavours of the region.': 'Découvrez les saveurs de la région.',
    'Stay comfortably and explore locally.': 'Séjournez confortablement et découvrez les environs.',
    'Discover more': 'En savoir plus',
    'Contact details': 'Coordonnées',
    'Reception Hours': 'Horaires de réception',
    '24 hours | 7 days': 'Arrivée automatique possible 24 h/24',
    'Concierge always available': 'Réception selon les horaires indiqués',
    'First name': 'Prénom',
    'Last name': 'Nom',
    'Email address': 'Adresse e-mail',
    'Phone number (optional)': 'Téléphone (facultatif)',
    'Send message': 'Envoyer le message',
    'How can we help?': 'Comment pouvons-nous vous aider ?',
    'We are always here.': 'Nous sommes à votre écoute.',
    'Check-In & Check-Out': 'Arrivée et départ',
    'Check-in is from 3pm. Check-out is by 11am.': "L'arrivée est possible à partir de 14 h. Le départ doit avoir lieu avant 11 h 30.",
    'Privacy Policy': 'Politique de confidentialité',
    'Terms & Conditions': 'Conditions générales',
    'Book now': 'Réserver',
    'Ask us': 'Nous consulter',
    'Availability': 'Disponibilité'
  };
  var reverse = {};
  Object.keys(translations).forEach(function (key) { reverse[translations[key]] = key; });
  function walk(node, map) {
    if (node.nodeType === 3) {
      var value = node.nodeValue.trim();
      if (/^from\s+(CHF|€|â)/i.test(value) || /^CHF\s*\d/i.test(value)) {
        node.nodeValue = node.nodeValue.replace(value, map === translations ? 'Disponibilité' : 'Availability');
        return;
      }
      if (/24 hours \| 7 days/i.test(value)) {
        node.nodeValue = node.nodeValue.replace(value, map === translations ? 'Arrivée automatique possible 24 h/24' : 'Automatic arrival available 24 hours a day');
        return;
      }
      if (/Alpine|Swiss|Lakeview|Geneva|butler|sommelier|champagne|Lumi(Ã|è)re Spa/i.test(value)) {
        node.nodeValue = node.nodeValue.replace(value, map === translations ? 'Services pratiques de l’hôtel' : 'Practical hotel services');
        return;
      }
      if (map[value]) node.nodeValue = node.nodeValue.replace(value, map[value]);
      return;
    }
    if (node.nodeType !== 1 || node.tagName === 'SCRIPT' || node.tagName === 'STYLE') return;
    Array.prototype.slice.call(node.childNodes).forEach(function (child) { walk(child, map); });
  }
  function setLanguage(lang) {
    var map = lang === 'fr' ? translations : reverse;
    walk(document.body, map);
    document.documentElement.lang = lang;
    document.querySelectorAll('[data-language-switch]').forEach(function (button) {
      button.textContent = lang === 'fr' ? 'EN' : 'FR';
      button.setAttribute('aria-label', lang === 'fr' ? 'Passer en anglais' : 'Passer en français');
    });
    localStorage.setItem('good-night-language', lang);
  }
  function init() {
    var button = document.createElement('button');
    button.type = 'button';
    button.dataset.languageSwitch = 'true';
    button.textContent = 'FR';
    button.style.cssText = 'position:fixed;z-index:2147483647;right:18px;bottom:18px;padding:10px 14px;border:1px solid currentColor;background:#fff;color:#1a1410;font:600 12px system-ui;cursor:pointer;';
    button.addEventListener('click', function () { setLanguage(document.documentElement.lang === 'fr' ? 'en' : 'fr'); });
    document.body.appendChild(button);
    setLanguage(localStorage.getItem('good-night-language') || 'en');
  }
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();
}());
