//
//  ViewController.swift
//  ARPoints
//
//  Created by Josh Robbins on 18/05/2018.
//  Copyright © 2018 BlackMirrorz. All rights reserved.
//

import UIKit
import ARKit
import FirebaseStorage
import FirebaseAuth
import Foundation
import ARDataLogger

extension ViewController: ARSCNViewDelegate{
    func renderer(_ renderer: SCNSceneRenderer, updateAtTime time: TimeInterval) {
        
        //1. Check Our Frame Is Valid & That We Have Received Our Raw Feature Points
        guard let currentFrame = self.augmentedRealitySession.currentFrame,
             let featurePointsArray = currentFrame.rawFeaturePoints?.points else { return }
        // Project all of the feature points (expressed in the global coordinate system) to pixels on the captured image
        let projectedFeaturePoints = featurePointsArray.map({feature in currentFrame.camera.projectPoint(feature, orientation: .landscapeRight, viewportSize: currentFrame.camera.imageResolution)})
        for featurePoint in featurePointsArray{
            let cameraCoords = currentFrame.camera.transform.inverse * simd_float4(featurePoint, 1)
        }
        
        if !sentData {
            sentData = true
            let fileType = StorageMetadata()
            fileType.contentType = "application/json"
            do {
                let jsonData: [String: Any] = ["projectedFeaturePoints": projectedFeaturePoints.map({[$0.x, $0.y]}), "featurePoints": featurePointsArray.map({[$0.x, $0.y, $0.z]})]
                let featurePointsData = try JSONSerialization.data(withJSONObject: jsonData, options: [])
                let ref = Storage.storage().reference().child("helloworld.json")
                ref.putData(featurePointsData, metadata: fileType) { (metadata, error) in
                    print("checking to see if it worked \(error) \(metadata)")
                }
                //(with: ["featurePoints": [[3.0, 2.0], [3.0, 3.0]]], options: [])
            } catch {
                print("error occurred")
            }
        }

        
        //2. Visualize The Feature Points
        visualizeFeaturePointsIn(featurePointsArray)
        
        //3. Update Our Status View
        DispatchQueue.main.async {
            
            //1. Update The Tracking Status
            self.statusLabel.text = self.augmentedRealitySession.sessionStatus()
            
            //2. If We Have Nothing To Report Then Hide The Status View & Shift The Settings Menu
            if let validSessionText = self.statusLabel.text{
                
                self.sessionLabelView.isHidden = validSessionText.isEmpty
            }
            
            if self.sessionLabelView.isHidden { self.settingsConstraint.constant = 26 } else { self.settingsConstraint.constant = 0 }
        }
    
    }
    
    /// Provides Visualization Of Raw Feature Points Detected In The ARSessopm
    ///
    /// - Parameter featurePointsArray: [vector_float3]
    func visualizeFeaturePointsIn(_ featurePointsArray: [vector_float3]){
        //1. Remove Any Existing Nodes
        self.augmentedRealityView.scene.rootNode.enumerateChildNodes { (featurePoint, _) in
            
           // featurePoint.geometry = nil
           // featurePoint.removeFromParentNode()
        }
        
        //2. Update Our Label Which Displays The Count Of Feature Points
        DispatchQueue.main.async {
            self.rawFeaturesLabel.text = self.Feature_Label_Prefix + String(featurePointsArray.count)
        }
        
        //3. Loop Through The Feature Points & Add Them To The Hierachy
        featurePointsArray.forEach { (pointLocation) in
            
            //Clone The SphereNode To Reduce CPU
            let clone = sphereNode.clone()
            clone.position = SCNVector3(pointLocation.x, pointLocation.y, pointLocation.z)
           // self.augmentedRealityView.scene.rootNode.addChildNode(clone)
        }

    }
  
}

class ViewController: UIViewController {
    var sentData = false
    var lastFrameUploadTime = Date()
    //1. Create A Reference To Our ARSCNView In Our Storyboard Which Displays The Camera Feed
    @IBOutlet weak var augmentedRealityView: ARSCNView!
    
    //2. Create A Reference To Our ARSCNView In Our Storyboard Which Will Display The ARSession Tracking Status
    @IBOutlet weak var sessionLabelView: UIView!
    @IBOutlet weak var statusLabel: UILabel!
    @IBOutlet weak var rawFeaturesLabel: UILabel!
    @IBOutlet var settingsConstraint: NSLayoutConstraint!
    var Feature_Label_Prefix = "Number Of Raw Feature Points Detected = "
    
    //3. Create Our ARWorld Tracking Configuration
    let configuration = ARWorldTrackingConfiguration()
    
    //4. Create Our Session
    let augmentedRealitySession = ARSession()
    
    //5. Create A Single SCNNode Which We Will Clone
    var sphereNode: SCNNode!
    
    //--------------------
    //MARK: View LifeCycle
    //--------------------
    
    override func viewDidLoad() {
        
        super.viewDidLoad()
        
        generateNode()
        setupARSession()

    }
    
    override var prefersStatusBarHidden: Bool { return true }

    override func didReceiveMemoryWarning() { super.didReceiveMemoryWarning() }

    //----------------------
    //MARK: SCNNode Creation
    //----------------------
    
    
    /// Generates A Spherical SCNNode
    func generateNode(){
        sphereNode = SCNNode()
        let sphereGeometry = SCNSphere(radius: 0.001)
        sphereGeometry.firstMaterial?.diffuse.contents = UIColor.green
        sphereNode.geometry = sphereGeometry
    }

    //---------------
    //MARK: ARSession
    //---------------
    
    /// Sets Up The ARSession
    func setupARSession(){
        ARDataLogger.ARLogger.shared.doAynchronousUploads = false
        ARDataLogger.ARLogger.shared.dataDir = "depth_benchmarking"
        ARDataLogger.ARLogger.shared.startTrial()
        //1. Set The AR Session
        augmentedRealityView.session = augmentedRealitySession
        augmentedRealityView.delegate = self
        augmentedRealityView.debugOptions = [.showFeaturePoints]
        
        configuration.planeDetection = [planeDetection(.None)]
        augmentedRealitySession.run(configuration, options: runOptions(.ResetAndRemove))
        augmentedRealitySession.delegate = self
        
        self.rawFeaturesLabel.text = ""
       
        
    }
}

extension ViewController: ARSessionDelegate {
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        ARDataLogger.ARLogger.shared.session(session, didUpdate: frame)
        if -lastFrameUploadTime.timeIntervalSinceNow > 2 {
            lastFrameUploadTime = Date()
            ARDataLogger.ARLogger.shared.log(frame: frame, withType: "depth_benchmarking", withMeshLoggingBehavior: .none)
        }
    }

    
    func session(_ session: ARSession, didAdd anchors: [ARAnchor]) {
        ARDataLogger.ARLogger.shared.session(session, didAdd: anchors)
    }
    
    func session(_ session: ARSession, didUpdate anchors: [ARAnchor]) {
        ARDataLogger.ARLogger.shared.session(session, didUpdate: anchors)
    }

    func session(_ session: ARSession, didRemove anchors: [ARAnchor]) {
        ARDataLogger.ARLogger.shared.session(session, didRemove: anchors)
    }
}
