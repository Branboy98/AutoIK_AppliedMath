# AutoIK_AppliedMath
  This Python 3 Script (compatible with Maya 2022 onward, only thing needed to use on older Maya version is changing print syntax) 
  takes the input of the first in the chain being selected (example being if you were to run it for a leg you would select the thigh joint), then creates a layer of 
  joints on top of the original joints, constrains the old joints to the new joints, then uses Applied Math to calculate the proper location to add a Locator for the
  IK that's then created for the new Joints. Lastly it creates a Control with two separate group layers for possible set driven keys which constrains the IK itself.
