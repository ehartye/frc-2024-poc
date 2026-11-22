(async () => {
  const frame = document.querySelector('#viewer');
  const direct = document.querySelector('#direct');
  const error = document.querySelector('#error');
  try {
    const response = await fetch('./releases.json');
    if (!response.ok) throw Error('The design index could not be loaded.');
    const {current, releases} = await response.json();
    if (releases.length !== 1 || releases[0] !== current || !/^[0-9]{3}$/.test(current))
      throw Error('The preliminary design index is invalid.');
    const page = new URL(location.href);
    const id = page.searchParams.get('release');
    if (id && !['001','002',current].includes(id)) throw Error('Unknown preliminary design.');
    const motion = page.searchParams.get('case');
    const showReplay = Boolean(motion);
    const url = new URL('releases/' + current + '/' + (showReplay ? 'match-replay.html' : ''), page);
    document.querySelector(showReplay ? '#replay' : '#assembly').setAttribute('aria-current','page');
    if (showReplay) url.searchParams.set('concept','guided');
    if (motion) {
      const earlier = {'loaded-cycle-48':'continuous-cycle-48','observed-reset-96':'continuous-cycle-96'};
      url.searchParams.set('case', earlier[motion] || motion);
    }
    page.searchParams.delete('release');
    if (motion) page.searchParams.set('case',url.searchParams.get('case'));
    history.replaceState(null,'',page);
    direct.href = url; direct.hidden = false; frame.src = url;
    addEventListener('message', e => {
      if (e.source !== frame.contentWindow || e.origin !== location.origin || e.data?.type !== 'review-case') return;
      const route = new URL(location.href);
      route.searchParams.set('case', e.data.case);
      history.replaceState(null, '', route);
      direct.href = frame.contentWindow.location.href;
    });
  } catch (e) { error.textContent=e.message; error.hidden=false; }
})();
