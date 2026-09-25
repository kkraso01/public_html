const doc=document.documentElement;
const nav=document.querySelector('.site-nav');
const glow=document.querySelector('.cursor-glow');
const modal=document.querySelector('#leadModal');
const heroCanvas=document.querySelector('.btn-hero-webgl canvas');

window.addEventListener('scroll',()=>nav?.classList.toggle('is-scrolled',window.scrollY>50),{passive:true});
window.addEventListener('mousemove',(event)=>{if(glow){glow.style.left=`${event.clientX}px`;glow.style.top=`${event.clientY}px`}} ,{passive:true});

document.querySelectorAll('a[href^="#"]').forEach((link)=>link.addEventListener('click',(event)=>{const target=document.querySelector(link.getAttribute('href'));if(target){event.preventDefault();target.scrollIntoView({behavior:'smooth',block:'start'})}}));

const observer=new IntersectionObserver((entries)=>entries.forEach((entry)=>{if(entry.isIntersecting){entry.target.classList.add('visible');observer.unobserve(entry.target)}}),{threshold:.15});
document.querySelectorAll('.reveal-clip-left,.reveal-slide-right,.reveal-pop,.reveal-zoom,.stagger-scale').forEach((element,index)=>{element.style.transitionDelay=`${Math.min(index%6,5)*70}ms`;observer.observe(element)});

document.querySelectorAll('.infra-card').forEach((card)=>card.addEventListener('mousemove',(event)=>{const rect=card.getBoundingClientRect();card.style.setProperty('--mouse-x',`${event.clientX-rect.left}px`);card.style.setProperty('--mouse-y',`${event.clientY-rect.top}px`)}));

document.querySelector('#themeToggle')?.addEventListener('click',()=>{const light=doc.classList.toggle('theme-light');doc.dataset.theme=light?'light':'dark';document.querySelector('#themeIcon').setAttribute('icon',light?'solar:moon-linear':'solar:sun-linear')});

function openModal(){modal?.classList.add('is-open');modal?.querySelector('input')?.focus()} function closeModal(){modal?.classList.remove('is-open')}
document.querySelectorAll('.btn,.nav-cta').forEach((button)=>button.addEventListener('click',(event)=>{if(button.tagName==='A'&&button.getAttribute('href')?.startsWith('#'))return;event.preventDefault();openModal()}));
document.querySelector('[data-close-modal]')?.addEventListener('click',closeModal);modal?.addEventListener('click',(event)=>{if(event.target===modal)closeModal()});document.addEventListener('keydown',(event)=>{if(event.key==='Escape')closeModal()});
document.querySelector('[data-lead-form]')?.addEventListener('submit',(event)=>{event.preventDefault();document.querySelector('[data-lead-status]').textContent='Спасибо! Это демонстрационная форма — подключите утверждённый канал связи перед запуском.'});

if(heroCanvas){const ctx=heroCanvas.getContext('2d');let t=0;const draw=()=>{const dpr=window.devicePixelRatio||1,w=heroCanvas.clientWidth,h=heroCanvas.clientHeight;if(heroCanvas.width!==w*dpr){heroCanvas.width=w*dpr;heroCanvas.height=h*dpr;ctx.scale(dpr,dpr)}ctx.clearRect(0,0,w,h);const gradient=ctx.createLinearGradient(0,0,w,h);gradient.addColorStop(0,'rgba(151,185,44,.5)');gradient.addColorStop(1,'rgba(84,73,66,.05)');ctx.fillStyle=gradient;ctx.fillRect(0,0,w,h);ctx.fillStyle='rgba(242,234,207,.45)';for(let i=0;i<22;i++){const x=(i/22)*w;const y=h*.54+Math.sin(t+i*.6)*7;ctx.beginPath();ctx.arc(x,y,1.5,0,Math.PI*2);ctx.fill()}t+=.035;requestAnimationFrame(draw)};draw()}
