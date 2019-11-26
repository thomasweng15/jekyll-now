![](https://travis-ci.org/thomasweng15/thomasweng15.github.io.svg?branch=release)

Run locally in two terminals: 
1. `JEKYLL_ENV=production bundle exec jekyll build --destination _site --safe --watch --drafts`
2. `cd _site && bundle exec jekyll serve` then visit `localhost:4000`

Deploy using Travis CI by pushing the `release` branch.


