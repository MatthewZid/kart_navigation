# Info

This package is a `template`. You don't have to fork it. Simply click on `Use this template` and a copy of it will be created under your username. This allows you to independently contribute to the repo without any history. Most importantly, you can have multiple such templated repos under your username to test different approaches.

## Branches

* The `competition` branch is the one you should use for ready-to-race code. A github action will be triggered each time you push to it, and run your code against a series of tests.
* The `dev` (or any other) branch is the one you can use to push code without "submitting" it for competition.

## Limitations

* You need to set a [secret](https://docs.github.com/en/actions/security-guides/encrypted-secrets#creating-encrypted-secrets-for-a-repository) for this repo, named `PAT`. This should be a [personal access token (PAT)](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) that has permissions for `public repos` (or the whole `repo` category for private repositories that use this template). This is required because the action for the leaderboards creates a fork under your username, in order to create a PR with your personal best for each track.
* As stated earlier, only code in the `competition` branch will be used for a place in the leaderboards.
* In order for your code to be successfully tested, you need to add all required nodes and configurations inside the navigation.launch.py file with all parameters set to their required values (no external remapping should be required). Your `navigation.launch.py` file should have the ability to accept a `track` parameter in case you need to load different nodes or configurations per-track. Check the action's launch command along with the equivalent launch file from the [ros2_kart_racing](https://github.com/gstavrinos/ros2_kart_racing) repo for specific info.
* The package should not be renamed. `kart_navigation` is your only option.
* Since the tests are executed with Github Actions, GUI is not available. Make sure you do not include any nodes or applications that require a GUI.
* Your robot needs to reach the finish line before the 30 minutes mark (sim time). The test fails otherwise. This is required to save resources, when robots get stuck.

