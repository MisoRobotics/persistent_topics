^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package persistent_topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* RAD-79: Migrate to Python 3
  Update to work with Python 3 and Noetic.
* Merge pull request `#3 <https://github.com/MisoRobotics/persistent_topics/issues/3>`_ from MisoRobotics/master
  Merge master back into develop
* Merge pull request `#1 <https://github.com/MisoRobotics/persistent_topics/issues/1>`_ from MisoRobotics/release/1.0.0
  Release/1.0.0
* Contributors: Nikita Kosolobov, Ryan Sinnet

1.0.0 (2018-06-13)
------------------
* Fixed fix for unconstructable stored message types
* Added explicit check for stored types that are no longer built
* Fixed incorrect multi-channel node behavior
* Ignored compiled Python files
* Demoted log level of many messages
* Separated single-channel and multi-channel topic behavior
* Added latched echoing to source_topics when specified
* Fixed issue with combination-conceptualized topics
* Fixed /tf_static issue, fixed topic_type_names bug
* Added use cases to README
* Populated README, fixed persistence format bug, improved launch demo
* Implemented first draft
* Initial commit
* Contributors: Benjamin Pelletier, BenjaminPelletier
