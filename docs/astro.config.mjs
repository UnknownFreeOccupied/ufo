// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// https://astro.build/config
export default defineConfig({
	site: 'https://unknownfreeoccupied.github.io',
  base: '/ufo',
	integrations: [
		starlight({
			title: {
				en: 'UFO',
				sv: 'UFO',
			},
			favicon: '/favicon.svg',
			social: [{ icon: 'github', label: 'GitHub', href: 'https://github.com/UnknownFreeOccupied/ufo' }],
			sidebar: [
				{ 
					label: 'Start Here',
					translations: {
						'sv-SE': 'Börja här',
					},
					items: [
						// Each item here is one entry in the navigation menu.
						'start_here/getting_started',
						'start_here/installation',
					]
				},
				{
					label: 'Concepts',
					translations: {
						'sv-SE': 'Koncept',
					},
					items: [
						// Each item here is one entry in the navigation menu.
						'concepts/core_concepts',
					],
				},
				{
					label: 'Components',
					translations: {
						'sv-SE': 'Komponenter',
					},
					autogenerate: { directory: 'components' },
					collapsed: true,
				},
				{
					label: 'Tutorials',
					translations: {
						'sv-SE': 'Handledningar',
					},
					items: [
						// Each item here is one entry in the navigation menu.
					],
				},
				{
					label: 'Integrations',
					translations: {
						'sv-SE': 'Integrationer',
					},
					items: [
						// Each item here is one entry in the navigation menu.
						'integrations/ros',
					],
				},
				{
					label: 'Guides',
					translations: {
						'sv-SE': 'Guider',
					},
					items: [
						// Each item here is one entry in the navigation menu.
						'guides/host_website',
					],
				},
				{
					label: 'Reference',
					translations: {
						'sv-SE': 'Referens',
					},
					items: [
						// Each item here is one entry in the navigation menu.
						{ label: 'C++ API', link: 'https://unknownfreeoccupied.github.io/ufo/cpp_api/' },
					]
				},
			],
			// Set English as the default language for this site.
      defaultLocale: 'en',
      locales: {
        // English docs in `src/content/docs/en/`
        en: {
          label: 'English',
					lang: 'en',
        },
        // Swedish docs in `src/content/docs/sv/`
        sv: {
          label: 'Svenska',
					lang: 'sv-SE',
        },
      },
		}),
	],
});
