(async () => {
  const select = document.querySelector('#release');
  const frame = document.querySelector('#viewer');
  const direct = document.querySelector('#direct');
  const error = document.querySelector('#error');
  function warn(message) { error.textContent = message; error.hidden = false; }
  try {
    const response = await fetch('./releases.json');
    if (!response.ok) throw new Error('The release index could not be loaded.');
    const {current: latest, releases} = await response.json();
    if (!Array.isArray(releases) || !releases.includes(latest) ||
        releases.some(id => typeof id !== 'string' || !/^[a-z0-9][a-z0-9-]{0,63}$/.test(id)) ||
        new Set(releases).size !== releases.length) throw new Error('The release index is invalid.');
    const catalog = new Map();
    for (const id of [...releases].reverse()) {
      const option = new Option(id + (id === latest ? ' (latest)' : ''), id);
      select.add(option);
      catalog.set(id, option);
      // An older record can be unavailable without delaying the selected replay.
      fetch('./releases/' + id + '/release.json').then(async response => {
        if (!response.ok) return;
        const record = await response.json();
        if (typeof record.title === 'string') option.textContent = id + ' · ' + record.title + (id === latest ? ' (latest)' : '');
      }).catch(() => {});
    }
    function load() {
      error.hidden = true;
      const query = new URLSearchParams(location.search);
      const id = query.get('release') ?? latest;
      if (!catalog.has(id)) {
        select.selectedIndex = -1;
        frame.removeAttribute('src'); direct.hidden = true;
        warn('Unknown release “' + id + '”. Choose an available release above.');
        return;
      }
      select.value = id;
      const url = new URL('releases/' + encodeURIComponent(id) + '/', location.href);
      url.searchParams.set('concept','guided');
      if (query.has('case')) url.searchParams.set('case', query.get('case'));
      direct.href = url; direct.hidden = false; frame.src = url;
    }
    select.onchange = () => {
      const url = new URL(location.href);
      url.searchParams.set('release', select.value); url.searchParams.delete('case');
      history.pushState(null,'',url); load();
    };
    addEventListener('popstate',load);
    addEventListener('message', e => {
      if (e.source !== frame.contentWindow || e.origin !== location.origin || !e.data || e.data.type !== 'review-case') return;
      const url = new URL(location.href); url.searchParams.set('case', e.data.case);
      history.replaceState(null,'',url); direct.href = frame.contentWindow.location.href;
    });
    load();
  } catch(e) { warn(e.message); }
})();
