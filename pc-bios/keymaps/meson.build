keymaps = {
  'ar': '-l ar',
  'bepo': '-l fr -v dvorak',
  'cz': '-l cz',
  'da': '-l dk',
  'de': '-l de -v nodeadkeys',
  'de-ch': '-l ch',
  'en-gb': '-l us',
  'en-us': '-l gb',
  'es': '-l es',
  'et': '-l et',
  'fi': '-l fi',
  'fo': '-l fo',
  'fr': '-l fr -v nodeadkeys',
  'fr-be': '-l be',
  'fr-ca': '-l ca -v fr',
  'fr-ch': '-l ch -v fr',
  'hr': '-l hr',
  'hu': '-l hu',
  'is': '-l is',
  'it': '-l it',
  'ja': '-l jp -m jp106',
  'lt': '-l lt',
  'lv': '-l lv',
  'mk': '-l mk',
  'nl': '-l nl',
  'no': '-l no',
  'pl': '-l pl',
  'pt': '-l pt',
  'pt-br': '-l br',
  'ru': '-l ru',
  'sl': '',
  'sv': '',
  'th': '-l th',
  'tr': '-l tr',
}

install_data(keymaps.keys(), install_dir: config_host['qemu_datadir'] / 'keymaps')

if meson.is_cross_build()
  native_qemu_keymap = find_program('qemu-keymap', required: false)
  have_qemu_keymap = native_qemu_keymap.found()
else
  native_qemu_keymap = qemu_keymap
  # disabler disables everything below
  have_qemu_keymap = true
endif
if have_qemu_keymap
  t = []
  foreach km, args: keymaps
    t += custom_target(km,
                       output: km,
                       command: [qemu_keymap, '-f', '@OUTPUT@', args.split()])
  endforeach
  if t.length() > 0
    alias_target('update-keymaps', t)
  endif
endif
