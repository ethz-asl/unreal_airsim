# Downloading UE4 Assets
## Available Assets
As of this writing, there's 4 asset packs available at ASL. 
All asset packs were downloaded from the Unreal Market Place on UE 4.24. 
These pac

**Indoor:**
* Flat - [ArchViz Furniture - Flat Pack](https://www.unrealengine.com/marketplace/en-US/product/archviz-furniture-flat-pack)
* StudioApartment - [Modern Studio Apartment](https://www.unrealengine.com/marketplace/en-US/product/modern-studio-apartment/reviews)

**Outdoor:**
* OldTown - [Old Town](https://www.unrealengine.com/marketplace/en-US/product/old-town)
* ModularNeighborhoodPack - [Modular Neighborhood Pack](https://www.unrealengine.com/marketplace/en-US/product/modular-neighborhood-pack)

## Downloading an Asset Pack
* The assets are stored on [GDrive](https://drive.google.com/drive/folders/1sad6Mr4RXUySrlpo2XLNs_GoyRkhRRth) at `Mapping/simulation/UE4_assets`. 
Please ask the mapping team for access to the required assets (.zip folder).
* Unzip the downloaded folder. Unfortunately, Ubuntu unzip is not extremely robust, if you get an error use jar:
  ```shell script
  sudo apt-get install fastjar
  jar xvf OldTown.zip
  ```

## Using the Asset Pack in the Unreal Editor
Recall that the asset packs are only a collection of assets and not full projects. 
They therefore can only be added to existing projects:
* If you want to create a new project, create one using the UE4 editor. 
  E.g. for a blank project use `File -> New Project... -> Game -> Blank`.
* Move or copy the entire asset folder **as is** into `.../UnrealProjects/MyProject/Content`.
* The assets can now be accessed from the 'Content Browser' in the UE4 Editor, where you can drag and drop assets into your level.
  Many packs come with a pre-built demo *level*, which can be opened via `File -> Open Level` or by double clicking it in the Content Browser.