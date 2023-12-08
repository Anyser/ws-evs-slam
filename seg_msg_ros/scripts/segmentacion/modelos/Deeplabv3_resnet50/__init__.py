#!/usr/bin/env python3

import numpy as np
import os
import torch 
import torchvision.transforms as transforms

PATH=os.path.dirname(__file__)
PATH_N="/Path/to/your/catkin_ws/src/seg_pytorch_ros/models/model_script.pt"
USE_GPU=True

class SegmentationModel(object):
    def __init__(self):
        self.model=torch.jit.load(PATH_N)
        self.device = torch.device("cuda" if USE_GPU and torch.cuda.is_available() else "cpu")	
        print("Using:", self.device)
        self.model.to(self.device)
	
        #print("Arquitectura del modelo", self.model)

        self.model.eval()

        self.preprocess= transforms.Compose([
             transforms.ToTensor(),
             transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),])
    
	
        sample_input = torch.rand(1, 3, 256, 512).to(self.device)
        self.traced_model = torch.jit.trace(self.model, sample_input)

    def infer(self, img):
        input_t = self.preprocess(img).unsqueeze(0).to(self.device)

        with torch.no_grad():
            output = self.traced_model(input_t)["out"][0]

        indices = torch.tensor([0, 7, 14, 15], device=self.device)
        outputn=torch.index_select(output, 0, indices)
 
        pred_mask=torch.argmax(outputn, axis=0)
        pred_mask = pred_mask.to(torch.uint8).squeeze().cpu().numpy()
        return pred_mask
