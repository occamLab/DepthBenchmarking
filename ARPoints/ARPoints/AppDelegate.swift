//
//  AppDelegate.swift
//  ARPoints
//
//  Created by Josh Robbins on 18/05/2018.
//  Copyright © 2018 BlackMirrorz. All rights reserved.
//

import UIKit
import FirebaseCore
import FirebaseStorage
import FirebaseAuth
@UIApplicationMain
class AppDelegate: UIResponder, UIApplicationDelegate {
    
    var window: UIWindow?
    
    
    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        FirebaseApp.configure()
        if Auth.auth().currentUser == nil {
            Auth.auth().signInAnonymously() { (authResult, error) in
                print("auth result \(authResult) \(error)")
            }
        }

        return true
    }
    
    func applicationWillResignActive(_ application: UIApplication) { }
    
    func applicationDidEnterBackground(_ application: UIApplication) { }
    
    func applicationWillEnterForeground(_ application: UIApplication) { }
    
    func applicationDidBecomeActive(_ application: UIApplication) { }
    
    func applicationWillTerminate(_ application: UIApplication) { }
    
}


