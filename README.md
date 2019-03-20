# The official Husarion documentation

### How to use locally

#### 1. Prerequisites

Ensure you have the latest version of [`Node`](https://nodejs.org/en/) installed. It is also recommended to install [`Yarn`](https://yarnpkg.com/lang/en/) as well.
* `Node` version >= 8.x
* `Yarn` version >= 1.5

#### 2. Install node modules

Open `docs_new` dir, switch to `website` subdir and run:

```bash
$ cd <installation_path>/docs_new/website && npm install # or yarn install
```

#### 3. Local build - images

In order to see pictures in your local build you have to comment line `37` in `<installation_path>/docs_new/website/siteConfig.js` file:

![image](https://user-images.githubusercontent.com/28567623/54622133-52a3d180-4a69-11e9-9c2e-8d4dc59b3bd7.png)

> **For contributors**
> 
> Remember to remove comment before pushing to master branch!

#### 4. Run documentation website locally

In `website` subdir run:

```bash
$ npm start # or yarn start
```

Load the example site at `http://localhost:3000` if it did not already open automatically.