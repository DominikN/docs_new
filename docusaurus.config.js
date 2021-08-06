module.exports = {
  title: "Husarion",
  tagline: "Autonomous Mobile Robots Made Simple",
  url: "https://husarion.com",
  baseUrl: "/",
  onBrokenLinks: "throw",
  onBrokenMarkdownLinks: "warn",
  favicon: "/img/favicon.ico",
  organizationName: "husarion", // Usually your GitHub org/user name.
  projectName: "husarion-docs", // Usually your repo name.

  themeConfig: {
    navbar: {
      // title: "husarion DOCS",
      logo: {
        alt: "Husarnet",
        srcDark: "/img/husarionDocLogo.png",
        src: "/img/husarion_logo.png",
        href: "https://husarion.com/",
      },
      items: [
        {
          to: "tutorials/",
          // activeBasePath: "docs",
          label: "Tutorials",
          position: "left",
        },
        {
          to: "manuals/",
          label: "Manuals",
          position: "left",
        },
        {
          to: "software/",
          label: "Software",
          position: "left",
        },
        {
          href: "https://github.com/husarion/",
          label: "GitHub",
          position: "right",
        },
      ],
    },
    footer: {
      style: "dark",
      links: [
        {
          title: "Company",
          items: [
            {
              label: "About Us",
              to: "https://husarion.com/about.html",
            },
            {
              label: "Contact",
              to: "https://husarion.com/contact.html",
            },
            {
              label: "Terms of Service",
              to: "https://cloud.husarion.com/tos",
            },
          ],
        },
        {
          title: "Developers",
          items: [
            {
              label: "Documentation",
              href: "https://husarion.com/manuals",
            },
            {
              label: "Community Forum",
              href: "https://community.husarion.com/",
            },
            {
              label: "Downloads",
              href: "https://husarion.com/downloads",
            },
          ],
        },
        {
          title: "Find us",
          items: [
            {
              label: "GitHub",
              href: "https://github.com/husarion",
            },
            {
              label: "Twitter",
              href: "https://twitter.com/husarion",
            },
            {
              label: "Facebook",
              href: "https://www.facebook.com/husarionTechnology/",
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Husarion sp. z o.o.`,
    },
  },
  presets: [
    [
      "@docusaurus/preset-classic",
      {
        docs: {
          path: 'docs/manuals',
          routeBasePath: 'manuals',
          sidebarPath: require.resolve("./sidebarsManuals.js"),
        },


        theme: {
          customCss: require.resolve("./src/css/custom.css"),
        },
      },
    ],
  ],
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'tutorials',
        path: 'docs/tutorials',
        routeBasePath: 'tutorials',
        sidebarPath: require.resolve('./sidebarsTutorials.js'),
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'software',
        path: 'docs/software',
        routeBasePath: 'software',
        sidebarPath: require.resolve('./sidebarsSoftware.js'),
      },
    ],
  ],
};
