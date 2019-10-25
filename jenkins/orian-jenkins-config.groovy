node {
  try {
    notifyBuild('STARTED')

    stage('Checkout') {
      if (env.BRANCH_NAME == 'master' || env.BRANCH_NAME == 'qa' || env.BRANCH_NAME == 'staging' ||env.BRANCH_NAME == 'develop') {
        echo 'Pulling...' + env.BRANCH_NAME
      	git url: 'https://go360-jenkins:Go360.io@github.com/go360-io/orion-ros2-reference.git', branch: env.BRANCH_NAME
        shortCommit = sh(returnStdout: true, script: "git log -n 1 --pretty=format:'%h'").trim()
        println(shortCommit)
      }
    }

    stage('Build Dev') {
      if (env.BRANCH_NAME == 'master' || env.BRANCH_NAME == 'qa' || env.BRANCH_NAME == 'staging' ||env.BRANCH_NAME == 'develop') {
        sh "cat ~/password.txt | docker login -u go360jenkins --password-stdin"
        sh "cd ./docker && pwd && AUTOMATED_BUILD=1 ./build_dev.sh"
      }
    }

    stage('Build ') {
      withEnv(["WS_KROS=/opt/ws_kros"]) {
        println(env.WS_KROS)
        sh "sudo unlink /opt/ws_kros && sudo ln -sf ${workspace} /opt/ws_kros"
        if (env.BRANCH_NAME == 'master' || env.BRANCH_NAME == 'qa' || env.BRANCH_NAME == 'staging' ||env.BRANCH_NAME == 'develop') {
          sh "pwd"
          sh "cd ./docker && ./build.sh"
        }
      }
    }

    stage('Upload S3') {
      if (env.BRANCH_NAME == 'master' || env.BRANCH_NAME == 'qa' || env.BRANCH_NAME == 'staging' ||env.BRANCH_NAME == 'develop') {
        //sh "rm -rf ./tmp && mkdir -p ./install/tmp"
        //sh "cd ./install && find ./rtcm/lib/ -name '*.so' | tar cvzf  ./tmp/rtcm.tar.gz -T - && find ./commonLib/lib/ -name '*.so' | tar cvzf  ./tmp/commonLib.tar.gz -T - && find ./rtcm_msgs/lib/ -name '*.so' | tar cvzf ./tmp/rtcm_msgs.tar.gz -T - && sudo find ./config/lib/ -name '*.so' | tar cvzf ./tmp/config.tar.gz -T -"
        sh "pwd && ls -ltr ./install/x64"
        sh "aws s3 sync ./install/x64 s3://go360-orion/builds/${shortCommit}/x64"
        sh "aws s3 sync ./install/x64 s3://go360-orion/builds/${env.BRANCH_NAME}/x64"
      }
    }

    stage("last-changes") {
      if (env.BRANCH_NAME == 'master' || env.BRANCH_NAME == 'qa' || env.BRANCH_NAME == 'staging' ||env.BRANCH_NAME == 'develop') {
        def publisher = LastChanges.getLastChangesPublisher "LAST_SUCCESSFUL_BUILD", "SIDE", "LINE", true, true, "", "", "", "", ""
          publisher.publishLastChanges()
          def changes = publisher.getLastChanges()
          println(changes.getEscapedDiff())
          def firstCommit = changes.getPreviousRevision().getCommitId();
          def lastCommit = changes.getCurrentRevision().getCommitId();
          for (commit in changes.getCommits()) {
            println(commit)
            def commitInfo = commit.getCommitInfo()
            println(commitInfo)
            println(commitInfo.getCommitMessage())
            println(commit.getChanges())
          }
        if (firstCommit != null) {
          def git_commit_config_msg = sh (script: "git log --oneline ${firstCommit}..${lastCommit} -- ./src/ConfigNode ", returnStdout: true).trim()
          def git_commit_rtcm_msg = sh (script: "git log --oneline ${firstCommit}..${lastCommit} -- ./src/RtcmNode ", returnStdout: true).trim()
          def git_commit_ubx_msg = sh (script: "git log --oneline ${firstCommit}..${lastCommit} -- ./src/UbxGpsNode ", returnStdout: true).trim()
          def git_commit_msg = sh ( script: "git log --oneline ${firstCommit}..${lastCommit}", returnStdout: true).trim()
          println("---------------------------------------------------");
          println(git_commit_config_msg);
          println(git_commit_rtcm_msg);
          println(git_commit_ubx_msg);
          println(git_commit_msg);
          println("---------------------------------------------------");

          if (!git_commit_config_msg.isEmpty()) {
            dir('CONFIG') {
              git url: 'https://go360-jenkins:Go360.io@github.com/go360-io/orion-ros2-config.git', branch: env.BRANCH_NAME
              sh "pwd && cp -Rf ../src/ConfigNode ./config_node && git add ./config_node && git commit -am \"${git_commit_config_msg}\" && git push origin ${env.BRANCH_NAME} "
            }
          }
          if (!git_commit_rtcm_msg.isEmpty()) {
            dir('RTCM') {
              git url: 'https://go360-jenkins:Go360.io@github.com/go360-io/orion-ros2-rtcm.git', branch: env.BRANCH_NAME
              sh "cp -Rf ../src/RtcmNode ./rtcm_node  && git add ./rtcm_node && git commit -am \"${git_commit_rtcm_msg}\" && git push origin ${env.BRANCH_NAME}"
            }
          }
          if (!git_commit_ubx_msg.isEmpty()) {
            dir('UBX') {
              git url: 'https://go360-jenkins:Go360.io@github.com/go360-io/orion-ros2-ublox.git', branch: env.BRANCH_NAME
              sh "cp -Rf ../src/UbxGpsNode ./ubx_node  && git add ./ubx_node && git commit -am \"${git_commit_ubx_msg}\" && git push origin ${env.BRANCH_NAME}"
            }
          }
        }
      }
      currentBuild.result = "SUCCESSFUL"
    }
 } catch (e) {
    // If there was an exception thrown, the build failed
    currentBuild.result = "FAILED"
    throw e
  } finally {
    // Success or failure, always send notifications
    notifyBuild(currentBuild.result)
  }
}

def notifyBuild(String buildStatus = 'STARTED') {
  // build status of null means successful
  buildStatus =  buildStatus ?: 'SUCCESSFUL'

  // Default values
  def colorName = 'RED'
  def colorCode = '#FF0000'
  def subject = "${buildStatus}: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]'"
  def summary = "${subject} (${env.BUILD_URL})"
  def details = """<p>STARTED: Job '${env.JOB_NAME} [${env.BUILD_NUMBER}]':</p>
    <p>Check console output at &QUOT;<a href='${env.BUILD_URL}'>${env.JOB_NAME} [${env.BUILD_NUMBER}]</a>&QUOT;</p>"""

  // Override default values based on build status
  if (buildStatus == 'STARTED') {
    color = 'YELLOW'
    colorCode = '#FFFF00'
  } else if (buildStatus == 'SUCCESSFUL') {
    color = 'GREEN'
    colorCode = '#00FF00'
  } else {
    color = 'RED'
    colorCode = '#FF0000'
  }

  // Send notifications
  slackSend(color: colorCode, message: summary)
}
